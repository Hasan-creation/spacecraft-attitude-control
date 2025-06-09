import matplotlib.animation as animation
import numpy as np
from matplotlib import pyplot as plt

from main import calculate_orientation_error, AttitudeController, create_rotation_matrix, vertices, edges


class SatelliteSimulation:
    """
    A physics-based simulation of a satellite with attitude control.
    This simulation models rotational inertia and applies control torques
    to demonstrate realistic satellite attitude dynamics.
    """

    def __init__(self, initial_roll=0, initial_pitch=0, initial_yaw=0):
        # Satellite's current state
        self.roll = initial_roll
        self.pitch = initial_pitch
        self.yaw = initial_yaw

        # Satellite's rotational velocities (how fast it's spinning around each axis)
        self.roll_rate = 0.0
        self.pitch_rate = 0.0
        self.yaw_rate = 0.0

        # Physical properties of our satellite
        self.inertia = 1.0  # Moment of inertia - resistance to rotation changes
        self.damping = 0.05  # Air resistance equivalent (energy dissipation)
        self.dt = 0.1  # Time step for simulation updates

        # Target orientation that we want to maintain
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        # Create our attitude controller
        self.controller = AttitudeController(kp=3.0, ki=0.2, kd=0.8)

        # Data storage for plotting performance over time
        self.time_history = []
        self.error_history = []
        self.control_history = []
        self.time = 0.0

    def apply_physics_update(self, torque_x, torque_y, torque_z):
        """
        Update the satellite's orientation based on applied torques and physics.
        This simulates how a real satellite would respond to control inputs.
        """

        # Newton's second law for rotation: Torque = Inertia * Angular_Acceleration
        # Therefore: Angular_Acceleration = Torque / Inertia
        roll_acceleration = torque_x / self.inertia
        pitch_acceleration = torque_y / self.inertia
        yaw_acceleration = torque_z / self.inertia

        # Apply damping to simulate energy dissipation (like atmospheric drag)
        self.roll_rate = self.roll_rate * (1 - self.damping) + roll_acceleration * self.dt
        self.pitch_rate = self.pitch_rate * (1 - self.damping) + pitch_acceleration * self.dt
        self.yaw_rate = self.yaw_rate * (1 - self.damping) + yaw_acceleration * self.dt

        # Update orientations based on current rotation rates
        self.roll += self.roll_rate * self.dt
        self.pitch += self.pitch_rate * self.dt
        self.yaw += self.yaw_rate * self.dt

        # Keep angles within reasonable bounds to avoid numerical issues
        self.roll = ((self.roll + 180) % 360) - 180
        self.pitch = ((self.pitch + 180) % 360) - 180
        self.yaw = ((self.yaw + 180) % 360) - 180

    def update_simulation(self):
        """
        Run one step of the simulation: sense, think, act, and update physics.
        This is the main control loop that runs continuously.
        """

        # Step 1: SENSE - Get the current orientation error
        axis_x, axis_y, axis_z, error_angle = calculate_orientation_error(
            self.roll, self.pitch, self.yaw,
            self.target_roll, self.target_pitch, self.target_yaw
        )

        # Step 2: THINK - Calculate what correction is needed
        torque_x, torque_y, torque_z = self.controller.update(
            self.roll, self.pitch, self.yaw,
            self.target_roll, self.target_pitch, self.target_yaw
        )

        # Step 3: ACT - Apply the calculated torques to the satellite
        self.apply_physics_update(torque_x, torque_y, torque_z)

        # Step 4: RECORD - Store data for analysis and visualization
        self.time += self.dt
        self.time_history.append(self.time)
        self.error_history.append(error_angle)
        self.control_history.append(np.sqrt(torque_x ** 2 + torque_y ** 2 + torque_z ** 2))

        return self.roll, self.pitch, self.yaw, error_angle

def create_attitude_simulation():
    """
    Creates a real-time simulation showing automatic attitude control in action.
    This simulation demonstrates how a PID controller maintains satellite orientation
    despite initial errors and ongoing disturbances.
    """

    # Initialize our satellite simulation with a significant initial error
    # This represents a satellite that has been knocked off its desired orientation
    sim = SatelliteSimulation(initial_roll=45, initial_pitch=30, initial_yaw=-60)

    # Set up the matplotlib figure with multiple subplots for comprehensive visualization
    fig = plt.figure(figsize=(15, 10))

    # Main 3D visualization of the satellite
    ax_3d = fig.add_subplot(2, 2, 1, projection='3d')
    ax_3d.set_title('Satellite Attitude Control Simulation', fontsize=14, fontweight='bold')
    ax_3d.set_xlabel('X-axis')
    ax_3d.set_ylabel('Y-axis')
    ax_3d.set_zlabel('Z-axis')

    # Performance monitoring plots
    ax_error = fig.add_subplot(2, 2, 2)
    ax_error.set_title('Orientation Error Over Time', fontsize=12)
    ax_error.set_xlabel('Time (seconds)')
    ax_error.set_ylabel('Error Angle (degrees)')
    ax_error.grid(True, alpha=0.3)

    ax_control = fig.add_subplot(2, 2, 3)
    ax_control.set_title('Control Effort Over Time', fontsize=12)
    ax_control.set_xlabel('Time (seconds)')
    ax_control.set_ylabel('Control Torque Magnitude')
    ax_control.grid(True, alpha=0.3)

    ax_angles = fig.add_subplot(2, 2, 4)
    ax_angles.set_title('Individual Angle Evolution', fontsize=12)
    ax_angles.set_xlabel('Time (seconds)')
    ax_angles.set_ylabel('Angle (degrees)')
    ax_angles.grid(True, alpha=0.3)

    # Storage for plot lines that we'll update during animation
    error_line, = ax_error.plot([], [], 'r-', linewidth=2, label='Orientation Error')
    control_line, = ax_control.plot([], [], 'b-', linewidth=2, label='Control Torque')
    roll_line, = ax_angles.plot([], [], 'r-', linewidth=2, label='Roll')
    pitch_line, = ax_angles.plot([], [], 'g-', linewidth=2, label='Pitch')
    yaw_line, = ax_angles.plot([], [], 'b-', linewidth=2, label='Yaw')

    # Add legends to help interpret the plots
    ax_error.legend()
    ax_control.legend()
    ax_angles.legend()

    # Storage for the 3D cube visualization elements
    cube_lines = []
    axis_lines = []

    def animate_frame(frame_number):
        """
        This function is called repeatedly to update the animation.
        Each call represents one step forward in time for our simulation.
        """

        # Clear the 3D plot for redrawing
        ax_3d.clear()
        ax_3d.set_title(f'Satellite Attitude Control - Time: {sim.time:.1f}s',
                        fontsize=14, fontweight='bold')
        ax_3d.set_xlabel('X-axis')
        ax_3d.set_ylabel('Y-axis')
        ax_3d.set_zlabel('Z-axis')

        # Add a disturbance every 50 frames to test controller robustness
        # This simulates external forces like solar wind or micro-meteorite impacts
        if frame_number > 0 and frame_number % 50 == 0:
            # Apply a random disturbance to test the controller's response
            disturbance_strength = 10.0  # degrees
            sim.roll += np.random.uniform(-disturbance_strength, disturbance_strength)
            sim.pitch += np.random.uniform(-disturbance_strength, disturbance_strength)
            sim.yaw += np.random.uniform(-disturbance_strength, disturbance_strength)
            print(f"Disturbance applied at time {sim.time:.1f}s - Testing controller resilience!")

        # Run one simulation step: the controller senses, thinks, and acts
        current_roll, current_pitch, current_yaw, error_angle = sim.update_simulation()

        # Create the rotated cube for visualization
        rotation_matrix = create_rotation_matrix(current_roll, current_pitch, current_yaw)
        rotated_vertices = vertices @ rotation_matrix.T

        # Draw the satellite cube with its current orientation
        for edge in edges:
            points = rotated_vertices[edge]
            ax_3d.plot([points[0, 0], points[1, 0]],
                       [points[0, 1], points[1, 1]],
                       [points[0, 2], points[1, 2]],
                       'b-', linewidth=2, alpha=0.8)

        # Draw the fixed reference coordinate system (space-fixed axes)
        ax_3d.plot([0, 2], [0, 0], [0, 0], 'r-', linewidth=3, alpha=0.7, label='Space X-axis')
        ax_3d.plot([0, 0], [0, 2], [0, 0], 'g-', linewidth=3, alpha=0.7, label='Space Y-axis')
        ax_3d.plot([0, 0], [0, 0], [0, 2], 'b-', linewidth=3, alpha=0.7, label='Space Z-axis')

        # Draw the satellite's current orientation axes (body-fixed axes)
        satellite_x = np.array([1, 0, 0]) @ rotation_matrix.T * 1.5
        satellite_y = np.array([0, 1, 0]) @ rotation_matrix.T * 1.5
        satellite_z = np.array([0, 0, 1]) @ rotation_matrix.T * 1.5

        ax_3d.plot([0, satellite_x[0]], [0, satellite_x[1]], [0, satellite_x[2]],
                   'r--', linewidth=2, alpha=0.9, label='Satellite X-axis')
        ax_3d.plot([0, satellite_y[0]], [0, satellite_y[1]], [0, satellite_y[2]],
                   'g--', linewidth=2, alpha=0.9, label='Satellite Y-axis')
        ax_3d.plot([0, satellite_z[0]], [0, satellite_z[1]], [0, satellite_z[2]],
                   'b--', linewidth=2, alpha=0.9, label='Satellite Z-axis')

        # Configure the 3D plot appearance
        ax_3d.set_xlim([-2.5, 2.5])
        ax_3d.set_ylim([-2.5, 2.5])
        ax_3d.set_zlim([-2.5, 2.5])
        ax_3d.legend(loc='upper right', fontsize=8)

        # Update performance monitoring plots with latest data
        if len(sim.time_history) > 1:  # Need at least 2 points to plot a line

            # Update error tracking plot
            error_line.set_data(sim.time_history, sim.error_history)
            ax_error.set_xlim(0, max(sim.time_history))
            ax_error.set_ylim(0, max(max(sim.error_history), 10))

            # Update control effort plot
            control_line.set_data(sim.time_history, sim.control_history)
            ax_control.set_xlim(0, max(sim.time_history))
            ax_control.set_ylim(0, max(max(sim.control_history), 1))

            # Update individual angle tracking
            roll_history = [sim.roll] * len(sim.time_history)  # Simplified for this demo
            pitch_history = [sim.pitch] * len(sim.time_history)
            yaw_history = [sim.yaw] * len(sim.time_history)

            roll_line.set_data(sim.time_history, roll_history)
            pitch_line.set_data(sim.time_history, pitch_history)
            yaw_line.set_data(sim.time_history, yaw_history)
            ax_angles.set_xlim(0, max(sim.time_history))
            ax_angles.set_ylim(-180, 180)

        # Display current status information
        status_text = f'Current: Roll={current_roll:.1f}°, Pitch={current_pitch:.1f}°, Yaw={current_yaw:.1f}°\n'
        status_text += f'Target: Roll={sim.target_roll:.1f}°, Pitch={sim.target_pitch:.1f}°, Yaw={sim.target_yaw:.1f}°\n'
        status_text += f'Error: {error_angle:.2f}°'

        # Add status text to the 3D plot
        ax_3d.text2D(0.02, 0.98, status_text, transform=ax_3d.transAxes,
                     fontsize=10, verticalalignment='top',
                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        return []

    # Create and start the animation
    # The interval parameter controls how fast the simulation runs (milliseconds between frames)
    anim = animation.FuncAnimation(fig, animate_frame, frames=200,
                                   interval=100, blit=False, repeat=True)

    # Display instructions for interaction
    print("ATTITUDE CONTROL SIMULATION STARTED")
    print("=" * 50)
    print("Watch as the satellite automatically corrects its orientation!")
    print("The controller will:")
    print("- Start with a large initial error (satellite misaligned)")
    print("- Apply corrective torques to reduce the error")
    print("- Maintain stability despite periodic disturbances")
    print("- Demonstrate the three components of PID control working together")
    print()
    print("Observe the performance plots to see:")
    print("- How quickly the error decreases over time")
    print("- How much control effort is required")
    print("- How the individual angles evolve toward their targets")
    print("=" * 50)

    plt.tight_layout()
    plt.show()

    return anim


# Launch the complete attitude control simulation
anim = create_attitude_simulation()