import pygame
import numpy as np
import math
import random
from dataclasses import dataclass
from typing import Tuple, List

def create_rotation_matrix(roll, pitch, yaw):
    roll = np.radians(roll)
    pitch = np.radians(pitch)
    yaw = np.radians(yaw)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    return Rz @ Ry @ Rx


def calculate_orientation_error(current_roll, current_pitch, current_yaw,
                                desired_roll, desired_pitch, desired_yaw):
    """Same function as before - copy from your main file"""
    R_current = create_rotation_matrix(current_roll, current_pitch, current_yaw)
    R_desired = create_rotation_matrix(desired_roll, desired_pitch, desired_yaw)

    R_error = R_desired @ R_current.T

    trace = np.trace(R_error)
    angle = np.arccos(np.clip((trace - 1) / 2, -1, 1))

    if angle > 1e-6:
        axis_x = (R_error[2, 1] - R_error[1, 2]) / (2 * np.sin(angle))
        axis_y = (R_error[0, 2] - R_error[2, 0]) / (2 * np.sin(angle))
        axis_z = (R_error[1, 0] - R_error[0, 1]) / (2 * np.sin(angle))

        axis_magnitude = np.sqrt(axis_x ** 2 + axis_y ** 2 + axis_z ** 2)
        if axis_magnitude > 1e-6:
            axis_x /= axis_magnitude
            axis_y /= axis_magnitude
            axis_z /= axis_magnitude
        else:
            axis_x, axis_y, axis_z = 0, 0, 1
            angle = 0
    else:
        axis_x, axis_y, axis_z = 0, 0, 1
        angle = 0

    angle_degrees = np.degrees(angle)
    return axis_x, axis_y, axis_z, angle_degrees


class AttitudeController:
    """Copy your AttitudeController class here with the same functionality"""

    def __init__(self, kp=1.0, ki=0.1, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.previous_error_axis = np.array([0.0, 0.0, 0.0])
        self.previous_error_angle = 0.0
        self.integral_error = np.array([0.0, 0.0, 0.0])
        self.dt = 0.016  # 60 FPS for smooth animation

    def update(self, current_roll, current_pitch, current_yaw,
               desired_roll, desired_pitch, desired_yaw):
        axis_x, axis_y, axis_z, error_angle = calculate_orientation_error(
            current_roll, current_pitch, current_yaw,
            desired_roll, desired_pitch, desired_yaw
        )

        error_vector = np.array([axis_x, axis_y, axis_z]) * np.radians(error_angle)

        proportional_term = self.kp * error_vector

        self.integral_error += error_vector * self.dt
        integral_term = self.ki * self.integral_error

        if hasattr(self, 'previous_error_vector'):
            error_rate = (error_vector - self.previous_error_vector) / self.dt
            derivative_term = self.kd * error_rate
        else:
            derivative_term = np.array([0.0, 0.0, 0.0])

        control_output = proportional_term + integral_term + derivative_term

        self.previous_error_vector = error_vector.copy()

        return control_output[0], control_output[1], control_output[2]


@dataclass
class Star:
    """Represents a background star with twinkling effect"""
    x: float
    y: float
    brightness: float
    twinkle_phase: float


class Particle:
    """Represents a thruster particle effect"""

    def __init__(self, x: float, y: float, vel_x: float, vel_y: float,
                 life: float, color: Tuple[int, int, int]):
        self.x = x
        self.y = y
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.life = life
        self.max_life = life
        self.color = color

    def update(self, dt: float):
        """Update particle position and fade over time"""
        self.x += self.vel_x * dt
        self.y += self.vel_y * dt
        self.life -= dt

    def is_alive(self) -> bool:
        return self.life > 0

    def get_alpha(self) -> float:
        """Get current transparency based on remaining life"""
        return max(0, self.life / self.max_life)


class SatelliteVisualization:
    """
    A beautiful PyGame visualization of satellite attitude control.
    This creates an immersive space environment that showcases
    the sophisticated control system we've built together.
    """

    def __init__(self, width: int = 1200, height: int = 800):
        pygame.init()

        # Display setup with a professional space-themed appearance
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Advanced Satellite Attitude Control System")

        # Color palette inspired by real space imagery
        self.SPACE_BLACK = (5, 5, 15)
        self.EARTH_BLUE = (70, 130, 180)
        self.EARTH_GREEN = (34, 139, 34)
        self.SATELLITE_SILVER = (192, 192, 192)
        self.SOLAR_PANEL_BLUE = (25, 25, 112)
        self.THRUSTER_ORANGE = (255, 140, 0)
        self.THRUSTER_YELLOW = (255, 255, 0)
        self.TEXT_WHITE = (255, 255, 255)
        self.TEXT_GREEN = (0, 255, 100)
        self.TEXT_RED = (255, 100, 100)

        # Initialize font system for elegant text displays
        try:
            self.font_large = pygame.font.Font(None, 36)
            self.font_medium = pygame.font.Font(None, 24)
            self.font_small = pygame.font.Font(None, 18)
        except:
            # Fallback to default font if custom fonts aren't available
            self.font_large = pygame.font.Font(pygame.font.get_default_font(), 36)
            self.font_medium = pygame.font.Font(pygame.font.get_default_font(), 24)
            self.font_small = pygame.font.Font(pygame.font.get_default_font(), 18)

        # Simulation state and control system
        self.satellite_attitude = [45.0, 30.0, -60.0]  # Initial misalignment
        self.satellite_rates = [0.0, 0.0, 0.0]  # Angular velocities
        self.target_attitude = [0.0, 0.0, 0.0]  # Desired orientation

        # Physical properties for realistic satellite dynamics
        self.inertia = 1.0
        self.damping = 0.02

        # Our sophisticated attitude controller
        self.controller = AttitudeController(kp=4.0, ki=0.3, kd=1.2)

        # Visual elements for immersive space environment
        self.earth_angle = 0.0  # Earth rotation for realism
        self.earth_center = (300, height - 200)
        self.earth_radius = 120

        self.satellite_pos = (width // 2, height // 2)
        self.satellite_size = 40

        # Create a beautiful starfield background
        self.stars = self.create_starfield(200)

        # Particle system for thruster visualization
        self.particles: List[Particle] = []

        # Performance tracking for real-time analysis
        self.error_history = []
        self.control_history = []
        self.time = 0.0
        self.clock = pygame.time.Clock()

        # Disturbance simulation for testing controller robustness
        self.disturbance_timer = 0.0
        self.disturbance_interval = 10.0  # Apply disturbance every 3 seconds

    def create_starfield(self, num_stars: int) -> List[Star]:
        """Generate a realistic starfield with twinkling effects"""
        stars = []
        for _ in range(num_stars):
            star = Star(
                x=random.uniform(0, self.width),
                y=random.uniform(0, self.height),
                brightness=random.uniform(0.3, 1.0),
                twinkle_phase=random.uniform(0, 2 * math.pi)
            )
            stars.append(star)
        return stars

    def update_starfield(self, dt: float):
        """Create the gentle twinkling effect that makes space feel alive"""
        for star in self.stars:
            star.twinkle_phase += dt * 2.0  # Slow, gentle twinkling

    def draw_starfield(self):
        """Render the starfield with realistic twinkling"""
        for star in self.stars:
            # Calculate twinkling brightness using sinusoidal variation
            twinkle_factor = 0.7 + 0.3 * math.sin(star.twinkle_phase)
            current_brightness = int(star.brightness * twinkle_factor * 255)

            # Draw stars with slight color variation for realism
            if star.brightness > 0.8:
                color = (current_brightness, current_brightness, min(255, current_brightness + 30))
            else:
                color = (current_brightness, current_brightness, current_brightness)

            pygame.draw.circle(self.screen, color, (int(star.x), int(star.y)), 1)

    def draw_earth(self):
        """
        Draw a photorealistic Earth with continental features, cloud patterns,
        and atmospheric effects. This creates an authentic space environment
        that immediately establishes the orbital context of our satellite simulation.
        """
        # Main Earth sphere with realistic color gradients
        # We'll build the Earth in layers, from the core outward to the atmosphere

        # Deep ocean base layer - the foundation of our planet
        for i in range(8):
            radius = self.earth_radius - i * 3
            # Create depth effect with gradually lighter blues toward the surface
            blue_intensity = 50 + i * 15
            color = (0, blue_intensity, min(180, blue_intensity + 60))
            if radius > 0:
                pygame.draw.circle(self.screen, color, self.earth_center, radius)

        # Continental landmasses with realistic positioning
        # These represent major continents visible from space
        self.earth_angle += 0.3  # Slower rotation for more realistic effect

        # Atmospheric halo effect - this creates the thin blue line visible from space
        # This effect gives depth and realism to our planetary representation
        for atmosphere_layer in range(5):
            atmosphere_radius = self.earth_radius + atmosphere_layer * 4
            atmosphere_intensity = 80 - atmosphere_layer * 15

            # Create the characteristic blue atmospheric glow
            atmosphere_color = (0, atmosphere_intensity // 3, atmosphere_intensity)

            # Draw atmospheric rings with decreasing opacity
            pygame.draw.circle(self.screen, atmosphere_color, self.earth_center, atmosphere_radius, 2)

        # Terminator line effect - the subtle shadow that shows day/night boundary
        # This adds another layer of realism by suggesting the three-dimensional nature of our planet
        shadow_offset_x = int(15 * math.cos(math.radians(self.earth_angle + 90)))
        shadow_offset_y = int(15 * math.sin(math.radians(self.earth_angle + 90)))

        # Create a subtle shadow gradient
        shadow_surface = pygame.Surface((self.earth_radius, self.earth_radius))
        shadow_surface.set_alpha(60)

        for shadow_ring in range(self.earth_radius // 4):
            shadow_intensity = shadow_ring * 2
            shadow_color = (shadow_intensity, shadow_intensity, shadow_intensity)
            pygame.draw.circle(shadow_surface, shadow_color,
                               (self.earth_radius // 2, self.earth_radius // 2),
                               self.earth_radius // 2 - shadow_ring)

        # Apply the shadow effect to create depth
        shadow_center = (self.earth_center[0] + shadow_offset_x, self.earth_center[1] + shadow_offset_y)
        shadow_rect = shadow_surface.get_rect(center=shadow_center)
        self.screen.blit(shadow_surface, shadow_rect)

    def draw_satellite(self, orientation: List[float], control_magnitude: float):
        """
        Draw a detailed satellite with orientation-dependent features.
        This visualization helps understand how the attitude control affects
        the satellite's physical orientation in space.
        """
        roll, pitch, yaw = orientation
        x, y = self.satellite_pos

        # Create rotation matrix for 2D projection of 3D orientation
        # We'll project the 3D rotation onto the 2D screen plane
        rotation_matrix = create_rotation_matrix(roll, pitch, yaw)

        # Define satellite body vertices in local coordinate system
        body_points = [
            (-self.satellite_size // 2, -self.satellite_size // 3),
            (self.satellite_size // 2, -self.satellite_size // 3),
            (self.satellite_size // 2, self.satellite_size // 3),
            (-self.satellite_size // 2, self.satellite_size // 3)
        ]

        # Transform points according to current orientation
        transformed_points = []
        for px, py in body_points:
            # Apply simplified 3D to 2D projection
            point_3d = np.array([px, py, 0])
            rotated_3d = point_3d @ rotation_matrix.T
            screen_x = x + int(rotated_3d[0])
            screen_y = y + int(rotated_3d[1])
            transformed_points.append((screen_x, screen_y))

        # Draw main satellite body
        pygame.draw.polygon(self.screen, self.SATELLITE_SILVER, transformed_points)
        pygame.draw.polygon(self.screen, (100, 100, 100), transformed_points, 2)

        # Draw solar panels that extend from the satellite body
        panel_length = self.satellite_size
        panel_width = 12

        # Left solar panel
        left_panel_base = np.array([-self.satellite_size // 2 - 5, 0, 0])
        left_panel_tip = np.array([-self.satellite_size // 2 - panel_length, 0, 0])

        left_base_rotated = (left_panel_base @ rotation_matrix.T)[:2] + np.array([x, y])
        left_tip_rotated = (left_panel_tip @ rotation_matrix.T)[:2] + np.array([x, y])

        pygame.draw.line(self.screen, self.SOLAR_PANEL_BLUE,
                         left_base_rotated.astype(int), left_tip_rotated.astype(int), panel_width)

        # Right solar panel
        right_panel_base = np.array([self.satellite_size // 2 + 5, 0, 0])
        right_panel_tip = np.array([self.satellite_size // 2 + panel_length, 0, 0])

        right_base_rotated = (right_panel_base @ rotation_matrix.T)[:2] + np.array([x, y])
        right_tip_rotated = (right_panel_tip @ rotation_matrix.T)[:2] + np.array([x, y])

        pygame.draw.line(self.screen, self.SOLAR_PANEL_BLUE,
                         right_base_rotated.astype(int), right_tip_rotated.astype(int), panel_width)

        # Draw attitude reference vectors to show current orientation
        vector_length = 60

        # X-axis (roll) - Red vector
        x_vector = np.array([vector_length, 0, 0]) @ rotation_matrix.T
        pygame.draw.line(self.screen, (255, 100, 100), (x, y),
                         (x + int(x_vector[0]), y + int(x_vector[1])), 3)

        # Y-axis (pitch) - Green vector
        y_vector = np.array([0, vector_length, 0]) @ rotation_matrix.T
        pygame.draw.line(self.screen, (100, 255, 100), (x, y),
                         (x + int(y_vector[0]), y + int(y_vector[1])), 3)

        # Z-axis (yaw) - Blue vector
        z_vector = np.array([0, 0, vector_length]) @ rotation_matrix.T
        pygame.draw.line(self.screen, (100, 100, 255), (x, y),
                         (x + int(z_vector[0]), y + int(z_vector[1])), 3)

        # Draw thruster effects when control is active
        if control_magnitude > 0.1:
            self.create_thruster_particles(x, y, orientation, control_magnitude)

    def create_thruster_particles(self, x: float, y: float,
                                  orientation: List[float], intensity: float):
        """
        Generate particle effects to visualize thruster activation.
        This provides immediate visual feedback about control system activity.
        """
        roll, pitch, yaw = orientation
        rotation_matrix = create_rotation_matrix(roll, pitch, yaw)

        # Create particles emanating from thruster locations
        num_particles = int(intensity * 5) + 2

        for _ in range(num_particles):
            # Thruster positions relative to satellite body
            thruster_positions = [
                np.array([0, self.satellite_size // 2, 0]),  # Bottom thruster
                np.array([0, -self.satellite_size // 2, 0]),  # Top thruster
                np.array([self.satellite_size // 2, 0, 0]),  # Right thruster
                np.array([-self.satellite_size // 2, 0, 0])  # Left thruster
            ]

            # Randomly select a thruster for this particle
            thruster_pos = random.choice(thruster_positions)
            world_pos = (thruster_pos @ rotation_matrix.T)[:2] + np.array([x, y])

            # Create particle with realistic physics
            angle = random.uniform(0, 2 * math.pi)
            speed = random.uniform(20, 80)
            vel_x = math.cos(angle) * speed
            vel_y = math.sin(angle) * speed

            # Color depends on thruster intensity
            if intensity > 2.0:
                color = self.THRUSTER_YELLOW
            else:
                color = self.THRUSTER_ORANGE

            particle = Particle(
                world_pos[0], world_pos[1],
                vel_x, vel_y,
                life=random.uniform(0.3, 0.8),
                color=color
            )
            self.particles.append(particle)

    def update_particles(self, dt: float):
        """Update and remove expired thruster particles"""
        self.particles = [p for p in self.particles if p.is_alive()]
        for particle in self.particles:
            particle.update(dt)

    def draw_particles(self):
        """Render thruster particle effects with proper alpha blending"""
        for particle in self.particles:
            alpha = particle.get_alpha()
            color = [int(c * alpha) for c in particle.color]

            # Draw particles with size based on remaining life
            size = max(1, int(alpha * 4))
            pygame.draw.circle(self.screen, color,(int(particle.x), int(particle.y)), size)

    def draw_performance_graphs(self):
        """
        Draw real-time performance metrics that demonstrate the effectiveness
        of our attitude control system. These graphs provide quantitative
        evidence of system performance that complements the visual simulation.
        """
        # Define the area for our performance dashboard
        graph_area_x = self.width - 350
        graph_area_y = 20
        graph_width = 320
        graph_height = 120

        # Background panel for professional appearance
        panel_surface = pygame.Surface((graph_width + 20, graph_height * 3 + 60))
        panel_surface.fill((20, 20, 40))
        panel_surface.set_alpha(220)  # Semi-transparent for modern look
        self.screen.blit(panel_surface, (graph_area_x - 10, graph_area_y - 10))

        # Error magnitude graph - shows how quickly the system converges
        if len(self.error_history) > 1:
            self.draw_line_graph(
                data=self.error_history[-100:],  # Last 100 data points
                x=graph_area_x, y=graph_area_y,
                width=graph_width, height=graph_height,
                title="Orientation Error (degrees)",
                color=self.TEXT_RED,
                max_value=max(max(self.error_history[-100:], default=1), 10)
            )

        # Control effort graph - shows how hard the controller is working
        if len(self.control_history) > 1:
            self.draw_line_graph(
                data=self.control_history[-100:],
                x=graph_area_x, y=graph_area_y + graph_height + 20,
                width=graph_width, height=graph_height,
                title="Control Effort",
                color=self.TEXT_GREEN,
                max_value=max(max(self.control_history[-100:], default=1), 5)
            )

        # System status indicators
        self.draw_status_indicators(graph_area_x, graph_area_y + 2 * graph_height + 40)

    def draw_line_graph(self, data: List[float], x: int, y: int,
                        width: int, height: int, title: str,
                        color: Tuple[int, int, int], max_value: float):
        """
        Draw a real-time line graph with professional styling.
        This creates the kind of monitoring displays you'd see in actual mission control.
        """
        # Graph background
        pygame.draw.rect(self.screen, (10, 10, 20), (x, y, width, height))
        pygame.draw.rect(self.screen, (60, 60, 80), (x, y, width, height), 2)

        # Title
        title_surface = self.font_small.render(title, True, self.TEXT_WHITE)
        self.screen.blit(title_surface, (x + 5, y + 5))

        # Grid lines for easier reading
        for i in range(1, 5):
            grid_y = y + (height * i // 5)
            pygame.draw.line(self.screen, (40, 40, 60),
                             (x, grid_y), (x + width, grid_y), 1)

        # Data visualization
        if len(data) > 1:
            points = []
            for i, value in enumerate(data):
                # Convert data coordinates to screen coordinates
                point_x = x + (i * width // len(data))
                point_y = y + height - int((value / max_value) * height)
                points.append((point_x, point_y))

            # Draw the performance curve
            if len(points) > 1:
                pygame.draw.lines(self.screen, color, False, points, 2)

            # Current value indicator
            current_value = data[-1]
            value_text = f"{current_value:.2f}"
            value_surface = self.font_small.render(value_text, True, color)
            self.screen.blit(value_surface, (x + width - 50, y + height - 20))

    def draw_status_indicators(self, x: int, y: int):
        """
        Display system status information that gives users immediate
        feedback about the health and performance of the control system.
        """
        # Calculate current system metrics
        current_error = self.error_history[-1] if self.error_history else 0
        current_control = self.control_history[-1] if self.control_history else 0

        # System status based on performance metrics
        if current_error < 1.0:
            status = "STABLE"
            status_color = self.TEXT_GREEN
        elif current_error < 5.0:
            status = "CONVERGING"
            status_color = (255, 255, 0)  # Yellow
        else:
            status = "CORRECTING"
            status_color = self.TEXT_RED

        # Status display
        status_text = f"System Status: {status}"
        status_surface = self.font_medium.render(status_text, True, status_color)
        self.screen.blit(status_surface, (x, y))

        # Detailed metrics
        metrics = [
            f"Error: {current_error:.2f}°",
            f"Control: {current_control:.2f}",
            f"Time: {self.time:.1f}s"
        ]

        for i, metric in enumerate(metrics):
            metric_surface = self.font_small.render(metric, True, self.TEXT_WHITE)
            self.screen.blit(metric_surface, (x, y + 30 + i * 20))

    def draw_orientation_display(self):
        """
        Create a heads-up display showing current and target orientations.
        This provides precise numerical feedback alongside the visual representation.
        """
        # HUD panel positioning
        hud_x = 20
        hud_y = 20

        # Semi-transparent background for readability
        hud_surface = pygame.Surface((250, 200))
        hud_surface.fill((20, 20, 40))
        hud_surface.set_alpha(200)
        self.screen.blit(hud_surface, (hud_x, hud_y))

        # Title
        title = "Attitude Control System"
        title_surface = self.font_large.render(title, True, self.TEXT_WHITE)
        self.screen.blit(title_surface, (hud_x + 10, hud_y + 10))

        # Current orientation display
        current_text = "Current Orientation:"
        current_surface = self.font_medium.render(current_text, True, self.TEXT_WHITE)
        self.screen.blit(current_surface, (hud_x + 10, hud_y + 50))

        roll, pitch, yaw = self.satellite_attitude
        orientation_lines = [
            f"Roll:  {roll:7.2f}°",
            f"Pitch: {pitch:7.2f}°",
            f"Yaw:   {yaw:7.2f}°"
        ]

        for i, line in enumerate(orientation_lines):
            line_surface = self.font_small.render(line, True, self.TEXT_GREEN)
            self.screen.blit(line_surface, (hud_x + 20, hud_y + 75 + i * 18))

        # Target orientation display
        target_text = "Target Orientation:"
        target_surface = self.font_medium.render(target_text, True, self.TEXT_WHITE)
        self.screen.blit(target_surface, (hud_x + 10, hud_y + 130))

        target_roll, target_pitch, target_yaw = self.target_attitude
        target_lines = [
            f"Roll:  {target_roll:7.2f}°",
            f"Pitch: {target_pitch:7.2f}°",
            f"Yaw:   {target_yaw:7.2f}°"
        ]

        for i, line in enumerate(target_lines):
            line_surface = self.font_small.render(line, True, self.TEXT_WHITE)
            self.screen.blit(line_surface, (hud_x + 20, hud_y + 155 + i * 18))

    def update_simulation(self, dt: float):
        """
        Execute one complete cycle of the satellite attitude control system.
        This method orchestrates the entire simulation: sensing, control computation,
        actuation, and physics update. It's the heart of our real-time system.
        """
        # Step 1: SENSE - Measure current orientation error
        axis_x, axis_y, axis_z, error_angle = calculate_orientation_error(
            self.satellite_attitude[0], self.satellite_attitude[1], self.satellite_attitude[2],
            self.target_attitude[0], self.target_attitude[1], self.target_attitude[2]
        )

        # Step 2: THINK - Compute optimal control response
        torque_x, torque_y, torque_z = self.controller.update(
            self.satellite_attitude[0], self.satellite_attitude[1], self.satellite_attitude[2],
            self.target_attitude[0], self.target_attitude[1], self.target_attitude[2]
        )

        # Step 3: ACT - Apply control torques to satellite dynamics
        self.apply_satellite_physics(torque_x, torque_y, torque_z, dt)

        # Step 4: RECORD - Store performance data for analysis
        self.error_history.append(error_angle)
        control_magnitude = math.sqrt(torque_x ** 2 + torque_y ** 2 + torque_z ** 2)
        self.control_history.append(control_magnitude)

        # Keep data history manageable for real-time performance
        if len(self.error_history) > 1000:
            self.error_history = self.error_history[-500:]
            self.control_history = self.control_history[-500:]

        # Step 5: SIMULATE DISTURBANCES - Test controller robustness
        self.disturbance_timer += dt
        if self.disturbance_timer >= self.disturbance_interval:
            self.apply_random_disturbance()
            self.disturbance_timer = 0.0

        # Update simulation time
        self.time += dt

        return control_magnitude

    def apply_satellite_physics(self, torque_x: float, torque_y: float,
                                torque_z: float, dt: float):
        """
        Simulate realistic satellite rotational dynamics.
        This models how a real satellite responds to control torques,
        including inertia effects and energy dissipation.
        """
        # Newton's law for rotation: Torque = Inertia × Angular Acceleration
        angular_accel = [
            torque_x / self.inertia,
            torque_y / self.inertia,
            torque_z / self.inertia
        ]

        # Update angular velocities with damping (simulates friction/drag)
        for i in range(3):
            self.satellite_rates[i] *= (1.0 - self.damping)
            self.satellite_rates[i] += angular_accel[i] * dt

        # Convert angular velocities to orientation changes
        # This uses Euler integration for simplicity (real systems use more sophisticated methods)
        attitude_change = [rate * dt * 180.0 / math.pi for rate in self.satellite_rates]

        # Update satellite orientation
        for i in range(3):
            self.satellite_attitude[i] += attitude_change[i]

        # Keep angles within reasonable bounds to prevent numerical drift
        for i in range(3):
            while self.satellite_attitude[i] > 180:
                self.satellite_attitude[i] -= 360
            while self.satellite_attitude[i] < -180:
                self.satellite_attitude[i] += 360

    def apply_random_disturbance(self):
        """
        Inject random disturbances to test the controller's ability to maintain stability.
        This simulates real-world perturbations like solar wind, atmospheric drag,
        or gravitational gradient torques that constantly affect satellites.
        """
        disturbance_strength = 60.0  # Maximum disturbance in degrees

        # Apply random perturbations to each axis
        for i in range(3):
            disturbance = random.uniform(-disturbance_strength, disturbance_strength)
            self.satellite_attitude[i] += disturbance

        print(f"[{self.time:.1f}s] Disturbance applied - Testing controller resilience!")

    def handle_user_interaction(self, event):
        """
        Process user input to allow interactive testing of the control system.
        This enables users to experiment with different scenarios and
        understand the system's capabilities and limitations.
        """
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                # Manual disturbance trigger
                self.apply_random_disturbance()
                print("Manual disturbance triggered!")

            elif event.key == pygame.K_r:
                # Reset simulation to initial conditions
                self.satellite_attitude = [45.0, 30.0, -60.0]
                self.satellite_rates = [0.0, 0.0, 0.0]
                self.controller = AttitudeController(kp=4.0, ki=0.3, kd=1.2)
                self.error_history.clear()
                self.control_history.clear()
                self.time = 0.0
                print("Simulation reset to initial conditions!")

            elif event.key == pygame.K_t:
                # Change target orientation for demonstration
                self.target_attitude = [
                    random.uniform(-30, 30),
                    random.uniform(-30, 30),
                    random.uniform(-30, 30)
                ]
                print(f"New target set: {self.target_attitude}")

    def run_simulation(self):
        """
        Main simulation loop that brings everything together.
        This orchestrates the entire real-time demonstration,
        creating a professional showcase of the attitude control system.
        """
        running = True

        print("=" * 60)
        print("ADVANCED SATELLITE ATTITUDE CONTROL DEMONSTRATION")
        print("=" * 60)
        print("Interactive Controls:")
        print("  SPACE - Apply random disturbance")
        print("  R     - Reset simulation")
        print("  T     - Set new random target")
        print("  ESC   - Exit simulation")
        print()
        print("Watch the satellite automatically stabilize its orientation!")
        print("Observe how the controller responds to disturbances.")
        print("=" * 60)

        while running:
            # Calculate frame time for consistent physics
            dt = self.clock.tick(60) / 1000.0  # 60 FPS, dt in seconds

            # Handle user input and system events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    else:
                        self.handle_user_interaction(event)

            # Update all simulation components
            control_magnitude = self.update_simulation(dt)
            self.update_starfield(dt)
            self.update_particles(dt)

            # Render the complete visualization
            self.screen.fill(self.SPACE_BLACK)

            # Background elements
            self.draw_starfield()
            self.draw_earth()

            # Main simulation elements
            self.draw_satellite(self.satellite_attitude, control_magnitude)
            self.draw_particles()

            # User interface elements
            self.draw_orientation_display()
            self.draw_performance_graphs()

            # Instructions overlay
            instructions = [
                "Controls: SPACE=Disturbance, R=Reset, T=New Target, ESC=Exit",
                "Watch the automatic attitude control system maintain stability!"
            ]

            for i, instruction in enumerate(instructions):
                text_surface = self.font_small.render(instruction, True, self.TEXT_WHITE)
                text_rect = text_surface.get_rect()
                text_rect.centerx = self.width // 2
                text_rect.y = self.height - 40 + i * 20

                # Semi-transparent background for readability
                bg_rect = text_rect.inflate(10, 2)
                bg_surface = pygame.Surface(bg_rect.size)
                bg_surface.fill((0, 0, 0))
                bg_surface.set_alpha(150)
                self.screen.blit(bg_surface, bg_rect)

                self.screen.blit(text_surface, text_rect)

            # Present the frame to the user
            pygame.display.flip()

        pygame.quit()


# Launch function for easy execution
def main():
    """
    Entry point for the satellite attitude control visualization.
    This creates and launches the complete interactive demonstration.
    """
    try:
        visualization = SatelliteVisualization()
        visualization.run_simulation()
    except Exception as e:
        print(f"Error running simulation: {e}")
        print("Make sure pygame is installed: pip install pygame")


if __name__ == "__main__":
    main()