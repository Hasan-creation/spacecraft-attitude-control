# 🌌 Physics Simulation for Spacecraft Attitude Control

## Overview

This document explores the physical principles and computational methods used to create realistic spacecraft attitude dynamics simulations. From Newton's laws of rotational motion to environmental disturbance modeling, these simulations provide the testing ground where control algorithms prove their effectiveness before deployment on actual space missions.

## Table of Contents

1. [Rigid Body Dynamics](#rigid-body-dynamics)
2. [Spacecraft Inertia Properties](#spacecraft-inertia-properties)
3. [Environmental Disturbances](#environmental-disturbances)
4. [Actuator Modeling](#actuator-modeling)
5. [Numerical Integration](#numerical-integration)
6. [Simulation Architecture](#simulation-architecture)

---

## Rigid Body Dynamics

### Euler's Equations of Motion

The fundamental equations governing spacecraft rotational motion are **Euler's equations**, derived from Newton's second law for rotational systems:

```
I₁ω̇₁ + (I₃ - I₂)ω₂ω₃ = T₁
I₂ω̇₂ + (I₁ - I₃)ω₃ω₁ = T₂  
I₃ω̇₃ + (I₂ - I₁)ω₁ω₂ = T₃
```

Where:
- **I₁, I₂, I₃**: Principal moments of inertia
- **ω₁, ω₂, ω₃**: Angular velocities around principal axes
- **T₁, T₂, T₃**: Applied torques around principal axes

### Physical Interpretation

**Gyroscopic Coupling**: The cross-terms (e.g., ω₂ω₃) represent gyroscopic effects where rotation around one axis creates apparent torques around other axes. This coupling is crucial for spacecraft dynamics.

**Example - Spinning Spacecraft**: A spacecraft spinning around its Z-axis will resist attempts to change its orientation around X or Y axes due to gyroscopic stiffness.

### Implementation in Python

```python
def euler_equations(state, torques, inertia_matrix):
    """
    Implement Euler's equations for spacecraft rotational dynamics.
    
    Args:
        state: [ω₁, ω₂, ω₃] angular velocities in body frame
        torques: [T₁, T₂, T₃] applied torques in body frame  
        inertia_matrix: 3x3 inertia tensor
    
    Returns:
        angular_acceleration: [ω̇₁, ω̇₂, ω̇₃]
    """
    omega = np.array(state)
    T = np.array(torques)
    I = inertia_matrix
    
    # Angular momentum in body frame
    H = I @ omega
    
    # Gyroscopic torque: ω × H
    gyroscopic_torque = np.cross(omega, H)
    
    # Angular acceleration: I⁻¹(T - ω × H)
    omega_dot = np.linalg.inv(I) @ (T - gyroscopic_torque)
    
    return omega_dot
```

### Attitude Kinematics

**Relationship between angular velocity and attitude change**:

For small angles, the attitude evolution can be approximated as:
```
θ̇ = ω × dt
```

For larger rotations, quaternion kinematics provide more accurate integration:
```
q̇ = 0.5 × Ω(ω) × q
```

Where Ω(ω) is the quaternion rate matrix.

---

## Spacecraft Inertia Properties

### Moment of Inertia Tensor

The **inertia tensor** characterizes how mass is distributed in a spacecraft, determining its resistance to rotational acceleration:

```
I = [Iₓₓ  Iₓᵧ  Iₓᵧ]
    [Iᵧₓ  Iᵧᵧ  Iᵧᵧ]  
    [Iᵧₓ  Iᵧᵧ  Iᵧᵧ]
```

**Diagonal terms**: Resistance to rotation around each axis
**Off-diagonal terms**: Coupling between rotational axes

### Principal Axes

Most spacecraft are designed so their **principal axes** align with geometric axes, making the inertia tensor diagonal:

```python
def calculate_spacecraft_inertia(mass_elements):
    """
    Calculate spacecraft inertia tensor from distributed mass elements.
    
    Args:
        mass_elements: List of (mass, position) tuples
    
    Returns:
        3x3 inertia tensor
    """
    I = np.zeros((3, 3))
    
    for mass, pos in mass_elements:
        x, y, z = pos
        
        # Diagonal terms
        I[0,0] += mass * (y**2 + z**2)  # Iₓₓ
        I[1,1] += mass * (x**2 + z**2)  # Iᵧᵧ  
        I[2,2] += mass * (x**2 + y**2)  # Iᵧᵧ
        
        # Off-diagonal terms
        I[0,1] -= mass * x * y  # Iₓᵧ
        I[0,2] -= mass * x * z  # Iₓᵧ
        I[1,2] -= mass * y * z  # Iᵧᵧ
    
    # Symmetry
    I[1,0] = I[0,1]
    I[2,0] = I[0,2] 
    I[2,1] = I[1,2]
    
    return I
```

### Typical Spacecraft Configurations

**CubeSat (3U)**:
```python
cubesat_inertia = np.diag([0.05, 0.05, 0.02])  # kg⋅m²
```

**Communications Satellite**:
```python
comsat_inertia = np.diag([1200, 1200, 800])  # kg⋅m²
```

**Space Station Module**:
```python
iss_module_inertia = np.diag([50000, 45000, 40000])  # kg⋅m²
```

### Inertia Variations

**Fuel Consumption**: As propellant is consumed, inertia properties change
**Deployable Structures**: Solar panel deployment significantly alters inertia
**Cargo Operations**: ISS inertia changes with visiting vehicle operations

```python
class VariableInertiaSpacecraft:
    """Spacecraft with time-varying inertia properties"""
    
    def __init__(self, dry_inertia, fuel_mass, fuel_geometry):
        self.dry_inertia = dry_inertia
        self.initial_fuel_mass = fuel_mass
        self.current_fuel_mass = fuel_mass
        self.fuel_geometry = fuel_geometry
        
    def current_inertia(self):
        """Calculate current inertia including remaining fuel"""
        fuel_inertia = self.calculate_fuel_inertia(self.current_fuel_mass)
        return self.dry_inertia + fuel_inertia
        
    def consume_fuel(self, delta_mass):
        """Update fuel mass and inertia"""
        self.current_fuel_mass = max(0, self.current_fuel_mass - delta_mass)
```

---

## Environmental Disturbances

### Atmospheric Drag (Low Earth Orbit)

**Physical Origin**: Collisions with atmospheric molecules create drag force and torque

**Mathematical Model**:
```
F_drag = -0.5 × ρ × v² × Cd × A × v̂
T_drag = r_cp × F_drag
```

Where:
- **ρ**: Atmospheric density
- **v**: Orbital velocity  
- **Cd**: Drag coefficient
- **A**: Cross-sectional area
- **r_cp**: Center of pressure offset from center of mass

**Implementation**:
```python
def atmospheric_drag_torque(altitude, velocity, spacecraft_geometry):
    """
    Calculate atmospheric drag disturbance torque.
    
    Args:
        altitude: Spacecraft altitude (km)
        velocity: Orbital velocity vector (m/s)
        spacecraft_geometry: Dictionary with drag properties
    
    Returns:
        torque_vector: Drag torque in body frame (N⋅m)
    """
    
    # Atmospheric density model (exponential approximation)
    def atmospheric_density(h):
        if h < 200:
            return 2.789e-10 * np.exp(-(h - 200) / 58.515)
        elif h < 500:
            return 5.464e-11 * np.exp(-(h - 500) / 58.515)
        else:
            return 3.019e-12 * np.exp(-(h - 500) / 58.515)
    
    rho = atmospheric_density(altitude)
    v_magnitude = np.linalg.norm(velocity)
    v_unit = velocity / v_magnitude
    
    # Drag force
    drag_force = -0.5 * rho * v_magnitude**2 * spacecraft_geometry['cd'] * spacecraft_geometry['area'] * v_unit
    
    # Torque from center of pressure offset
    cp_offset = spacecraft_geometry['center_of_pressure_offset']
    drag_torque = np.cross(cp_offset, drag_force)
    
    return drag_torque
```

### Solar Radiation Pressure

**Physical Origin**: Photon momentum transfer from solar radiation

**Solar Constant**: 1361 W/m² at 1 AU from the Sun

**Mathematical Model**:
```
F_srp = (Φ/c) × A × (1 + ρ) × cos(θ) × ŝ
```

Where:
- **Φ**: Solar flux (W/m²)
- **c**: Speed of light
- **A**: Effective area
- **ρ**: Surface reflectivity (0 = absorbing, 1 = reflecting)
- **θ**: Angle between surface normal and sun vector

**Implementation**:
```python
def solar_radiation_pressure_torque(sun_vector, spacecraft_surfaces):
    """
    Calculate solar radiation pressure torque for multi-surface spacecraft.
    
    Args:
        sun_vector: Unit vector pointing toward sun
        spacecraft_surfaces: List of surface dictionaries
    
    Returns:
        total_torque: SRP torque vector (N⋅m)
    """
    
    solar_flux = 1361  # W/m² at 1 AU
    c = 299792458  # m/s
    
    total_torque = np.zeros(3)
    
    for surface in spacecraft_surfaces:
        # Surface properties
        area = surface['area']
        normal = surface['normal_vector']
        reflectivity = surface['reflectivity']
        position = surface['position_vector']
        
        # Incident angle
        cos_theta = max(0, np.dot(normal, sun_vector))
        
        if cos_theta > 0:  # Surface is illuminated
            # Solar pressure force
            pressure = solar_flux / c
            force_magnitude = pressure * area * (1 + reflectivity) * cos_theta
            force_vector = force_magnitude * normal
            
            # Torque contribution
            torque_contribution = np.cross(position, force_vector)
            total_torque += torque_contribution
    
    return total_torque
```

### Magnetic Dipole Interaction

**Physical Origin**: Interaction between spacecraft magnetic dipole and Earth's magnetic field

**Earth's Magnetic Field Model** (simplified dipole):
```python
def earth_magnetic_field(position_eci, time):
    """
    Calculate Earth's magnetic field at spacecraft position.
    
    Args:
        position_eci: Spacecraft position in ECI frame (m)
        time: Current time (for field rotation)
    
    Returns:
        B_field: Magnetic field vector (Tesla)
    """
    
    # Earth's magnetic dipole moment
    M_earth = 7.94e22  # A⋅m²
    mu_0 = 4e-7 * np.pi  # Permeability of free space
    
    # Distance from Earth center
    r = np.linalg.norm(position_eci)
    r_unit = position_eci / r
    
    # Magnetic dipole axis (approximately aligned with geographic north)
    dipole_axis = np.array([0, 0, 1])  # Simplified
    
    # Dipole field formula
    B_magnitude = (mu_0 * M_earth) / (4 * np.pi * r**3)
    B_field = B_magnitude * (3 * np.dot(dipole_axis, r_unit) * r_unit - dipole_axis)
    
    return B_field

def magnetic_dipole_torque(spacecraft_dipole, earth_b_field):
    """
    Calculate torque from magnetic dipole interaction.
    
    Args:
        spacecraft_dipole: Spacecraft magnetic dipole moment (A⋅m²)
        earth_b_field: Earth's magnetic field vector (Tesla)
    
    Returns:
        magnetic_torque: Torque vector (N⋅m)
    """
    
    # Torque = μ × B
    magnetic_torque = np.cross(spacecraft_dipole, earth_b_field)
    
    return magnetic_torque
```

### Gravity Gradient Torque

**Physical Origin**: Differential gravitational force across spacecraft extent

**Mathematical Formulation**:
```
T_gg = (3μ/r³) × (r̂ × I × r̂)
```

Where:
- **μ**: Earth's gravitational parameter
- **r**: Position vector from Earth center
- **I**: Spacecraft inertia tensor

**Implementation**:
```python
def gravity_gradient_torque(position_eci, spacecraft_inertia):
    """
    Calculate gravity gradient disturbance torque.
    
    Args:
        position_eci: Spacecraft position vector from Earth center (m)
        spacecraft_inertia: 3x3 inertia tensor (kg⋅m²)
    
    Returns:
        gg_torque: Gravity gradient torque (N⋅m)
    """
    
    mu_earth = 3.986004418e14  # m³/s²
    
    r = np.linalg.norm(position_eci)
    r_unit = position_eci / r
    
    # Gravity gradient coefficient
    gg_coeff = 3 * mu_earth / r**3
    
    # Torque calculation: 3μ/r³ × (r̂ × I × r̂)
    I_times_r = spacecraft_inertia @ r_unit
    gg_torque = gg_coeff * np.cross(r_unit, I_times_r)
    
    return gg_torque
```

---

## Actuator Modeling

### Reaction Wheel Dynamics

**Physical Principle**: Conservation of angular momentum - spinning wheels store momentum

**Mathematical Model**:
```python
class ReactionWheel:
    """Model for spacecraft reaction wheel actuator"""
    
    def __init__(self, axis, max_torque, max_momentum, inertia):
        self.axis = axis / np.linalg.norm(axis)  # Unit vector
        self.max_torque = max_torque  # N⋅m
        self.max_momentum = max_momentum  # N⋅m⋅s
        self.wheel_inertia = inertia  # kg⋅m²
        self.current_momentum = 0.0
        
    def apply_torque(self, commanded_torque, dt):
        """
        Apply torque command and update wheel momentum.
        
        Args:
            commanded_torque: Desired torque along wheel axis (N⋅m)
            dt: Time step (s)
        
        Returns:
            actual_torque: Actually applied torque (N⋅m)
        """
        
        # Torque saturation
        actual_torque = np.clip(commanded_torque, -self.max_torque, self.max_torque)
        
        # Update wheel momentum
        momentum_change = actual_torque * dt
        new_momentum = self.current_momentum + momentum_change
        
        # Momentum saturation
        if abs(new_momentum) > self.max_momentum:
            # Wheel saturated - can't provide full torque
            available_momentum = self.max_momentum - abs(self.current_momentum)
            actual_torque = np.sign(commanded_torque) * available_momentum / dt
            new_momentum = np.sign(new_momentum) * self.max_momentum
        
        self.current_momentum = new_momentum
        
        # Torque applied to spacecraft (reaction)
        spacecraft_torque = -actual_torque * self.axis
        
        return spacecraft_torque, actual_torque
    
    def is_saturated(self, threshold=0.9):
        """Check if wheel is approaching saturation"""
        return abs(self.current_momentum) > threshold * self.max_momentum
```

### Thruster Modeling

**Physical Characteristics**: Discrete impulse generation with minimum impulse bits

```python
class Thruster:
    """Model for spacecraft thruster actuator"""
    
    def __init__(self, position, direction, thrust_level, min_impulse_bit):
        self.position = position  # Position vector from CM (m)
        self.direction = direction / np.linalg.norm(direction)  # Unit vector
        self.thrust_level = thrust_level  # Thrust magnitude (N)
        self.min_impulse_bit = min_impulse_bit  # Minimum impulse (N⋅s)
        self.fuel_consumption_rate = 1e-6  # kg/N⋅s
        
    def generate_impulse(self, requested_impulse):
        """
        Generate thruster impulse with quantization effects.
        
        Args:
            requested_impulse: Desired impulse magnitude (N⋅s)
        
        Returns:
            actual_impulse: Quantized actual impulse (N⋅s)
            fuel_consumed: Propellant mass consumed (kg)
        """
        
        # Quantize to minimum impulse bit
        num_pulses = round(requested_impulse / self.min_impulse_bit)
        actual_impulse = num_pulses * self.min_impulse_bit
        
        # Calculate resulting force and torque
        impulse_vector = actual_impulse * self.direction
        torque_vector = np.cross(self.position, impulse_vector)
        
        # Fuel consumption
        fuel_consumed = actual_impulse * self.fuel_consumption_rate
        
        return impulse_vector, torque_vector, fuel_consumed
```

### Control Moment Gyroscope (CMG)

**Advanced Actuator**: Variable-speed control moment gyroscopes for large spacecraft

```python
class ControlMomentGyroscope:
    """Model for Control Moment Gyroscope (CMG) actuator"""
    
    def __init__(self, gimbal_axis, rotor_inertia, rotor_speed):
        self.gimbal_axis = gimbal_axis / np.linalg.norm(gimbal_axis)
        self.rotor_inertia = rotor_inertia  # kg⋅m²
        self.rotor_speed = rotor_speed  # rad/s
        self.gimbal_angle = 0.0  # Current gimbal angle
        self.max_gimbal_rate = np.pi / 4  # rad/s
        
    def generate_torque(self, commanded_gimbal_rate, dt):
        """
        Generate torque through gimbal motion.
        
        CMG torque is perpendicular to both rotor angular momentum
        and gimbal rotation axis.
        """
        
        # Limit gimbal rate
        actual_gimbal_rate = np.clip(commanded_gimbal_rate, 
                                   -self.max_gimbal_rate, 
                                   self.max_gimbal_rate)
        
        # Update gimbal angle
        self.gimbal_angle += actual_gimbal_rate * dt
        
        # Rotor angular momentum vector
        rotor_momentum_magnitude = self.rotor_inertia * self.rotor_speed
        
        # Rotor direction (perpendicular to gimbal axis)
        rotor_direction = np.array([np.cos(self.gimbal_angle), 
                                  np.sin(self.gimbal_angle), 0])
        rotor_momentum = rotor_momentum_magnitude * rotor_direction
        
        # CMG torque = dH/dt = Ω × H
        cmg_torque = actual_gimbal_rate * np.cross(self.gimbal_axis, rotor_momentum)
        
        return cmg_torque
```

---

## Numerical Integration

### Integration Methods for Spacecraft Dynamics

**Euler Integration** (First-order, simple but inaccurate):
```python
def euler_integration(state, state_derivative, dt):
    """Simple Euler integration step"""
    return state + state_derivative * dt
```

**Runge-Kutta 4th Order** (Fourth-order, good accuracy):
```python
def rk4_integration(state, dynamics_function, dt, *args):
    """
    Fourth-order Runge-Kutta integration for spacecraft dynamics.
    
    Args:
        state: Current state vector [attitude, angular_velocity]
        dynamics_function: Function computing state derivative
        dt: Time step
        *args: Additional arguments for dynamics function
    
    Returns:
        new_state: Updated state after time step
    """
    
    # RK4 coefficients
    k1 = dynamics_function(state, *args)
    k2 = dynamics_function(state + 0.5 * dt * k1, *args)
    k3 = dynamics_function(state + 0.5 * dt * k2, *args)
    k4 = dynamics_function(state + dt * k3, *args)
    
    # Weighted average
    state_derivative = (k1 + 2*k2 + 2*k3 + k4) / 6
    new_state = state + dt * state_derivative
    
    return new_state
```

### Quaternion Integration

**Special consideration for attitude**: Quaternions must remain unit magnitude

```python
def integrate_quaternion(quaternion, angular_velocity, dt):
    """
    Integrate quaternion kinematics while preserving unit magnitude.
    
    Args:
        quaternion: Current attitude quaternion [w, x, y, z]
        angular_velocity: Body-frame angular velocity [rad/s]
        dt: Time step
    
    Returns:
        new_quaternion: Updated unit quaternion
    """
    
    # Quaternion rate matrix
    omega = angular_velocity
    Omega = 0.5 * np.array([
        [0, -omega[0], -omega[1], -omega[2]],
        [omega[0], 0, omega[2], -omega[1]],
        [omega[1], -omega[2], 0, omega[0]],
        [omega[2], omega[1], -omega[0], 0]
    ])
    
    # Quaternion derivative
    q_dot = Omega @ quaternion
    
    # Integration (using matrix exponential for accuracy)
    q_new = quaternion + q_dot * dt
    
    # Normalize to maintain unit magnitude
    q_new = q_new / np.linalg.norm(q_new)
    
    return q_new
```

### Adaptive Step Size Control

**Optimize accuracy vs computational cost**:

```python
class AdaptiveIntegrator:
    """Adaptive step size integrator for spacecraft simulation"""
    
    def __init__(self, max_error=1e-6, min_dt=1e-4, max_dt=0.1):
        self.max_error = max_error
        self.min_dt = min_dt
        self.max_dt = max_dt
        
    def integrate_step(self, state, dynamics_func, dt_nominal, *args):
        """
        Perform integration with adaptive step size control.
        """
        
        dt = dt_nominal
        
        while True:
            # Full step
            state_full = rk4_integration(state, dynamics_func, dt, *args)
            
            # Two half steps
            state_half1 = rk4_integration(state, dynamics_func, dt/2, *args)
            state_half2 = rk4_integration(state_half1, dynamics_func, dt/2, *args)
            
            # Error estimate
            error = np.linalg.norm(state_full - state_half2)
            
            if error < self.max_error or dt <= self.min_dt:
                # Accept step
                return state_half2, dt
            else:
                # Reduce step size
                dt = max(dt * 0.5, self.min_dt)
```

---

## Simulation Architecture

### Complete Spacecraft Simulation Class

```python
class SpacecraftSimulation:
    """
    Comprehensive spacecraft attitude dynamics simulation.
    Integrates all physical effects for realistic behavior.
    """
    
    def __init__(self, spacecraft_config, orbital_config, environment_config):
        
        # Spacecraft properties
        self.inertia_matrix = spacecraft_config['inertia']
        self.actuators = spacecraft_config['actuators']
        self.mass_properties = spacecraft_config['mass_properties']
        
        # Initial conditions
        self.attitude_quaternion = spacecraft_config['initial_attitude']
        self.angular_velocity = spacecraft_config['initial_rates']
        
        # Orbital environment
        self.orbital_elements = orbital_config
        self.environment = environment_config
        
        # Simulation state
        self.time = 0.0
        self.dt = 0.1  # Default time step
        
        # Data storage
        self.history = {
            'time': [], 'attitude': [], 'angular_velocity': [],
            'control_torques': [], 'disturbance_torques': []
        }
        
    def compute_disturbance_torques(self):
        """Calculate all environmental disturbance torques"""
        
        total_disturbance = np.zeros(3)
        
        # Update orbital position
        position_eci = self.compute_orbital_position()
        velocity_eci = self.compute_orbital_velocity()
        
        # Atmospheric drag
        if self.orbital_elements['altitude'] < 800:  # km
            drag_torque = atmospheric_drag_torque(
                self.orbital_elements['altitude'],
                velocity_eci,
                self.environment['drag_properties']
            )
            total_disturbance += drag_torque
        
        # Solar radiation pressure
        sun_vector_eci = self.compute_sun_vector()
        srp_torque = solar_radiation_pressure_torque(
            sun_vector_eci,
            self.environment['surface_properties']
        )
        total_disturbance += srp_torque
        
        # Gravity gradient
        gg_torque = gravity_gradient_torque(position_eci, self.inertia_matrix)
        total_disturbance += gg_torque
        
        # Magnetic dipole
        if hasattr(self.environment, 'magnetic_dipole'):
            b_field = earth_magnetic_field(position_eci, self.time)
            mag_torque = magnetic_dipole_torque(
                self.environment['magnetic_dipole'], b_field
            )
            total_disturbance += mag_torque
        
        return total_disturbance
    
    def update_dynamics(self, control_torques, dt):
        """
        Update spacecraft state using Euler's equations.
        
        Args:
            control_torques: Applied control torques (N⋅m)
            dt: Time step (s)
        """
        
        # Calculate disturbance torques
        disturbance_torques = self.compute_disturbance_torques()
        
        # Total applied torques
        total_torques = control_torques + disturbance_torques
        
        # Angular acceleration from Euler's equations
        omega_dot = euler_equations(
            self.angular_velocity, 
            total_torques, 
            self.inertia_matrix
        )
        
        # Integrate angular velocity
        self.angular_velocity += omega_dot * dt
        
        # Integrate attitude quaternion
        self.attitude_quaternion = integrate_quaternion(
            self.attitude_quaternion,
            self.angular_velocity,
            dt
        )
        
        # Update simulation time
        self.time += dt
        
        # Store history
        self.history['time'].append(self.time)
        self.history['attitude'].append(self.attitude_quaternion.copy())
        self.history['angular_velocity'].append(self.angular_velocity.copy())
        self.history['control_torques'].append(control_torques.copy())
        self.history['disturbance_torques'].append(disturbance_torques.copy())
    
    def run_simulation(self, duration, controller, target_attitude):
        """
        Execute complete simulation with closed-loop control.
        
        Args:
            duration: Simulation time (s)
            controller: Attitude controller object
            target_attitude: Desired spacecraft attitude
        """
        
        steps = int(duration / self.dt)
        
        for step in range(steps):
            
            # Convert quaternion to Euler angles for controller
            current_euler = quaternion_to_euler(self.attitude_quaternion)
            target_euler = quaternion_to_euler(target_attitude)
            
            # Compute control torques
            control_torques, debug_info = controller.update(
                current_euler, target_euler, self.dt
            )
            
            # Apply actuator models
            actual_torques = self.apply_actuator_models(control_torques)
            
            # Update spacecraft dynamics
            self.update_dynamics(actual_torques, self.dt)
            
            # Optional: Add noise and sensor models
            if hasattr(self, 'sensor_noise'):
                self.add_measurement_noise()
        
        return self.history
    
    def apply_actuator_models(self, commanded_torques):
        """Apply realistic actuator limitations and dynamics"""
        
        actual_torques = np.zeros(3)
        
        for actuator in self.actuators:
            if actuator.type == 'reaction_wheel':
                # Project commanded torque onto wheel axis
                wheel_torque_command = np.dot(commanded_torques, actuator.axis)
                spacecraft_torque, _ = actuator.apply_torque(wheel_torque_command, self.dt)
                actual_torques += spacecraft_torque
                
            elif actuator.type == 'thruster':
                # Thruster logic (simplified)
                if np.linalg.norm(commanded_torques) > actuator.threshold:
                    thruster_impulse = commanded_torques * self.dt
                    impulse_vec, torque_vec, fuel = actuator.generate_impulse(
                        np.linalg.norm(thruster_impulse)
                    )
                    actual_torques += torque_vec / self.dt
        
        return actual_torques
```

### Monte Carlo Analysis

**Statistical validation** of control system performance:

```python
def monte_carlo_analysis(num_runs=1000):
    """
    Perform Monte Carlo analysis of spacecraft attitude control system.
    Varies initial conditions, disturbances, and parameters.
    """
    
    results = {
        'settling_times': [],
        'max_errors': [],
        'control_efforts': [],
        'success_rate': 0
    }
    
    for run in range(num_runs):
        
        # Randomize initial conditions
        initial_attitude = random_quaternion()
        initial_rates = np.random.normal(0, 0.1, 3)  # rad/s
        
        # Randomize disturbance magnitudes
        disturbance_scale = np.random.uniform(0.5, 2.0)
        
        # Randomize spacecraft properties
        inertia_uncertainty = np.random.normal(1.0, 0.1, 3)
        
        # Create simulation with random parameters
        sim = SpacecraftSimulation(
            spacecraft_config={
                'initial_attitude': initial_attitude,
                'initial_rates': initial_rates,
                'inertia': base_inertia * inertia_uncertainty,
                # ... other config
            }
        )
        
        # Run simulation
        try:
            history = sim.run_simulation(duration=300, controller=pid_controller, 
                                       target_attitude=np.array([1,0,0,0]))
            
            # Analyze results
            settling_time = calculate_settling_time(history)
            max_error = np.max(history['attitude_errors'])
            control_effort = np.sum(np.array(history['control_torques'])**2)
            
            results['settling_times'].append(settling_time)
            results['max_errors'].append(max_error)
            results['control_efforts'].append(control_effort)
            
            if settling_time < 100 and max_error < 5.0:  # Success criteria
                results['success_rate'] += 1
                
        except Exception as e:
            # Simulation failed - unstable
            print(f"Run {run} failed: {e}")
    
    results['success_rate'] /= num_runs
    
    return results
```

---

## Conclusion

Physics simulation forms the critical bridge between theoretical control algorithms and real-world spacecraft performance. Accurate modeling of spacecraft dynamics, environmental disturbances, and actuator limitations enables engineers to validate control systems before committing to expensive space missions.

The progression from simple rigid body equations to comprehensive multi-physics simulations reflects the increasing sophistication required for modern space missions. Today's spacecraft simulators must account for dozens of coupled effects: flexible body dynamics, propellant sloshing, thermal distortions, and degrading component performance over multi-year missions.

Advanced simulation techniques like Monte Carlo analysis enable statistical validation of system robustness, while high-fidelity models support mission-critical decisions from design through operations. The International Space Station, Mars rovers, and deep space missions all rely on extensive simulation validation before deployment.

Understanding these physics simulation principles is essential for spacecraft engineers, providing the foundation for designing control systems that work reliably in the challenging environment of space. The mathematical elegance of orbital mechanics, combined with the practical constraints of real hardware, creates one of the most intellectually satisfying fields in aerospace engineering.

### Further Reading

- **Orbital Mechanics**: "Fundamentals of Astrodynamics" by Bate, Mueller, and White
- **Spacecraft Dynamics**: "Spacecraft Dynamics and Control" by de Ruiter, Damaren, and Forbes  
- **Numerical Methods**: "Numerical Recipes" by Press, Teukolsky, Vetterling, and Flannery
- **Environmental Models**: NASA technical reports on space environment characterization