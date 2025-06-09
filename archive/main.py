import matplotlib
matplotlib.use('TkAgg')

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


def create_rotation_matrix(roll, pitch, yaw):
    """
    Creates a 3D rotation matrix from Euler angles
    roll: rotation around X-axis (degrees) - like tilting head left/right
    pitch: rotation around Y-axis (degrees) - like nodding head up/down
    yaw: rotation around Z-axis (degrees) - like turning head left/right
    """
    # Convert degrees to radians (trigonometric functions expect radians)
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    # Rotation matrix around X-axis (roll)
    # This matrix keeps X constant and rotates Y and Z coordinates
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Rotation matrix around Y-axis (pitch)
    # This matrix keeps Y constant and rotates X and Z coordinates
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation matrix around Z-axis (yaw)
    # This matrix keeps Z constant and rotates X and Y coordinates
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # CRITICAL ORDER: Rz @ Ry @ Rx means "first roll, then pitch, then yaw"
    # In mathematics, matrix multiplication reads RIGHT to LEFT
    # So Rz @ Ry @ Rx = "apply Rx, then Ry, then Rz"
    #
    # Why this order? Think of an aircraft:
    # 1. First it banks its wings (roll)
    # 2. Then it climbs or dives (pitch)
    # 3. Finally it turns left or right (yaw)
    #
    # Different order = completely different final orientation!
    # It's like getting dressed: socks then shoes ≠ shoes then socks!
    return Rz @ Ry @ Rx


def analyze_rotation_capability(roll, pitch, yaw):
    """
    Better analysis of gimbal lock by checking multiple points
    and looking at the determinant of the rotation matrix
    """
    # Get rotation matrices for small changes
    delta = 1.0

    base_matrix = create_rotation_matrix(roll, pitch, yaw)
    roll_matrix = create_rotation_matrix(roll + delta, pitch, yaw)
    pitch_matrix = create_rotation_matrix(roll, pitch + delta, yaw)
    yaw_matrix = create_rotation_matrix(roll, pitch, yaw + delta)

    # Test multiple points to get a better picture
    test_points = np.array([
        [1, 0, 0],  # X-axis
        [0, 1, 0],  # Y-axis
        [0, 0, 1],  # Z-axis
        [1, 1, 0],  # Diagonal in XY plane
    ])

    total_roll_movement = 0
    total_yaw_movement = 0

    for point in test_points:
        base_point = point @ base_matrix.T
        roll_point = point @ roll_matrix.T
        yaw_point = point @ yaw_matrix.T

        roll_movement = np.linalg.norm(roll_point - base_point)
        yaw_movement = np.linalg.norm(yaw_point - base_point)

        total_roll_movement += roll_movement
        total_yaw_movement += yaw_movement

    # Another way to detect gimbal lock: check if the rotation matrices
    # for roll and yaw changes produce similar effects
    roll_diff = roll_matrix @ np.linalg.inv(base_matrix)
    yaw_diff = yaw_matrix @ np.linalg.inv(base_matrix)

    # Check how similar these differential rotations are
    similarity = np.trace(roll_diff @ yaw_diff.T) / 3.0  # Normalized trace

    print(f"Angles: Roll={roll}°, Pitch={pitch}°, Yaw={yaw}°")
    print(f"Total movements - Roll: {total_roll_movement:.4f}, Yaw: {total_yaw_movement:.4f}")
    print(f"Matrix similarity: {similarity:.4f}")
    print(f"Gimbal lock risk: {abs(abs(similarity) - 1.0):.4f} (closer to 0 = higher risk)")
    print("-" * 50)

    return similarity


def calculate_orientation_error(current_roll, current_pitch, current_yaw,
                                desired_roll, desired_pitch, desired_yaw):
    """
    Calculates the orientation error between current and desired orientations.
    Instead of simply subtracting angles (which doesn't work in 3D),
    this function finds the single rotation that would bring the object
    from its current orientation to the desired orientation.

    Returns the error as axis-angle representation: (axis_x, axis_y, axis_z, angle)
    where the axis is the direction around which to rotate, and angle is how much.
    """
    # Get the rotation matrices for current and desired orientations
    R_current = create_rotation_matrix(current_roll, current_pitch, current_yaw)
    R_desired = create_rotation_matrix(desired_roll, desired_pitch, desired_yaw)

    # Calculate the error rotation matrix
    # This represents the rotation needed to go from current to desired orientation
    R_error = R_desired @ R_current.T  # R_current.T is the inverse of R_current

    # Convert the error rotation matrix to axis-angle representation
    # This gives us the most direct path to correct the orientation

    # Extract the rotation angle from the trace of the matrix
    # The trace (sum of diagonal elements) is related to the rotation angle
    trace = np.trace(R_error)
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))  # Clamp to avoid numerical errors

    # Extract the rotation axis from the skew-symmetric part of the matrix
    # The rotation axis is encoded in the off-diagonal elements
    if angle > 1e-6:  # If there's a significant rotation needed
        # Calculate the rotation axis components
        axis_x = (R_error[2, 1] - R_error[1, 2]) / (2 * np.sin(angle))
        axis_y = (R_error[0, 2] - R_error[2, 0]) / (2 * np.sin(angle))
        axis_z = (R_error[1, 0] - R_error[0, 1]) / (2 * np.sin(angle))

        # Normalize the axis to make it a unit vector
        axis_magnitude = np.sqrt(axis_x ** 2 + axis_y ** 2 + axis_z ** 2)
        if axis_magnitude > 1e-6:
            axis_x /= axis_magnitude
            axis_y /= axis_magnitude
            axis_z /= axis_magnitude
        else:
            # If axis magnitude is too small, default to no rotation
            axis_x, axis_y, axis_z = 0, 0, 1
            angle = 0
    else:
        # If the angle is very small, there's essentially no error
        axis_x, axis_y, axis_z = 0, 0, 1  # Default axis direction
        angle = 0

    # Convert angle from radians to degrees for easier interpretation
    angle_degrees = np.degrees(angle)

    return axis_x, axis_y, axis_z, angle_degrees


class AttitudeController:
    """
    A PID controller specifically designed for 3D attitude control.
    This controller takes orientation errors in axis-angle representation
    and computes the necessary control torques to correct the attitude.
    """

    def __init__(self, kp=1.0, ki=0.1, kd=0.05):
        """
        Initialize the PID controller with tunable gains.

        kp (Proportional gain): How aggressively to respond to current error
        ki (Integral gain): How much to compensate for accumulated past errors
        kd (Derivative gain): How much to anticipate future error trends
        """
        self.kp = kp  # Proportional gain - immediate response to error
        self.ki = ki  # Integral gain - memory of persistent errors
        self.kd = kd  # Derivative gain - prediction of future behavior

        # Internal state variables to track error history
        self.previous_error_axis = np.array([0.0, 0.0, 0.0])
        self.previous_error_angle = 0.0
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.dt = 0.1  # Time step for simulation (100ms updates)

    def update(self, current_roll, current_pitch, current_yaw,
               desired_roll, desired_pitch, desired_yaw):
        """
        Calculate the control output needed to correct orientation error.

        This function implements the complete PID control law using the
        axis-angle error representation we developed earlier.

        Returns: (torque_x, torque_y, torque_z) - control torques to apply
        """

        # Step 1: Calculate the current orientation error
        axis_x, axis_y, axis_z, error_angle = calculate_orientation_error(
            current_roll, current_pitch, current_yaw,
            desired_roll, desired_pitch, desired_yaw
        )

        # Convert the axis-angle error into a 3D error vector
        # This vector points in the direction of needed correction
        # with magnitude proportional to the error angle
        error_vector = np.array([axis_x, axis_y, axis_z]) * np.radians(error_angle)

        # Step 2: Proportional term - immediate response to current error
        # The bigger the error, the stronger the correction
        proportional_term = self.kp * error_vector

        # Step 3: Integral term - accumulated memory of past errors
        # This helps eliminate steady-state errors and persistent biases
        self.integral_error += error_vector * self.dt
        integral_term = self.ki * self.integral_error

        # Step 4: Derivative term - prediction based on error rate of change
        # This helps prevent overshoot and reduces oscillations
        if hasattr(self, 'previous_error_vector'):
            error_rate = (error_vector - self.previous_error_vector) / self.dt
            derivative_term = self.kd * error_rate
        else:
            derivative_term = np.array([0.0, 0.0, 0.0])  # No derivative on first run

        # Step 5: Combine all three terms to get the total control output
        control_output = proportional_term + integral_term + derivative_term

        # Step 6: Store current values for next iteration
        self.previous_error_vector = error_vector.copy()

        # Step 7: Return the control torques needed around each axis
        return control_output[0], control_output[1], control_output[2]

    def reset(self):
        """
        Reset the controller's internal state.
        Useful when starting a new control sequence or after a disturbance.
        """
        self.integral_error = np.array([0.0, 0.0, 0.0])
        if hasattr(self, 'previous_error_vector'):
            delattr(self, 'previous_error_vector')

# Define the original cube vertices
vertices = np.array([
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]
])

edges = [
    [0,1], [1,2], [2,3], [3,0],  # bottom face
    [4,5], [5,6], [6,7], [7,4],  # top face
    [0,4], [1,5], [2,6], [3,7]   # vertical edges
]

test_angles = [
    (30, 0, 30),  # Normal case
    (30, 45, 30),  # Halfway to gimbal lock
    (30, 89, 30),  # Very close to gimbal lock
    (30, 90, 30),  # Gimbal lock!
    (30, 91, 30),  # Just past gimbal lock
]

axis_x, axis_y, axis_z, angle = calculate_orientation_error(0, 0, 0, 30, 45, 60)
print(f"Test 3 - Complex rotation: Axis=({axis_x:.3f}, {axis_y:.3f}, {axis_z:.3f}), Angle={angle:.3f}°")



print("GIMBAL LOCK ANALYSIS")
print("=" * 50)

for roll, pitch, yaw in test_angles:
    similarity = analyze_rotation_capability(roll, pitch, yaw)

    # Still show the visual
    rotation_matrix = create_rotation_matrix(roll, pitch, yaw)
    rotated_vertices = vertices @ rotation_matrix.T

    # Create a separate figure for each test case
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # Draw the cube
    for edge in edges:
        points = rotated_vertices[edge]
        ax.plot([points[0, 0], points[1, 0]],
                [points[0, 1], points[1, 1]],
                [points[0, 2], points[1, 2]], 'b-', linewidth=2)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'Pitch={pitch}° (Gimbal Lock Indicator: {abs(similarity):.3f})')

    plt.show()

# Test our attitude controller
print("TESTING ATTITUDE CONTROLLER")
print("=" * 50)

# Create a controller instance with moderate gains
controller = AttitudeController(kp=2.0, ki=0.5, kd=0.1)

# Test scenario: cube is at (30, 45, 60) and we want it at (0, 0, 0)
current_orientation = (30, 45, 60)
desired_orientation = (0, 0, 0)

print(f"Current orientation: {current_orientation}")
print(f"Desired orientation: {desired_orientation}")
print()

# Calculate what the controller suggests as correction
torque_x, torque_y, torque_z = controller.update(
    current_orientation[0], current_orientation[1], current_orientation[2],
    desired_orientation[0], desired_orientation[1], desired_orientation[2]
)

print(f"Controller output:")
print(f"Torque X: {torque_x:.4f}")
print(f"Torque Y: {torque_y:.4f}")
print(f"Torque Z: {torque_z:.4f}")
print(f"Total torque magnitude: {np.sqrt(torque_x**2 + torque_y**2 + torque_z**2):.4f}")