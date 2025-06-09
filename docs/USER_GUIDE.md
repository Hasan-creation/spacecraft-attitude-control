# 📖 User Guide - Spacecraft Attitude Control System

## Overview

This comprehensive user guide provides step-by-step instructions for using the spacecraft attitude control simulation system. Whether you're a student learning about control theory, an engineer validating algorithms, or an educator demonstrating aerospace concepts, this guide will help you get the most from the simulation.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Running the Demonstrations](#running-the-demonstrations)
3. [Understanding the Interface](#understanding-the-interface)
4. [Interactive Controls](#interactive-controls)
5. [Customizing Simulations](#customizing-simulations)
6. [Educational Applications](#educational-applications)
7. [Advanced Features](#advanced-features)
8. [Troubleshooting](#troubleshooting)
9. [Best Practices](#best-practices)

---

## Getting Started

### System Requirements

**Minimum Requirements:**
- Python 3.8 or higher
- 4 GB RAM
- Graphics support for 2D/3D visualization
- 100 MB free disk space

**Recommended Configuration:**
- Python 3.9 or higher
- 8 GB RAM
- Dedicated graphics card
- Multi-core processor for smooth real-time simulation

### Installation

**Step 1: Download the Project**
```bash
# Option A: Clone from GitHub
git clone https://github.com/teyk0o/spacecraft-attitude-control.git
cd spatial-systems-learning

# Option B: Download ZIP file and extract
# Then navigate to the extracted folder
```

**Step 2: Install Dependencies**
```bash
# Install required packages
pip install -r requirements.txt

# Verify installation
python -c "import numpy, matplotlib, pygame; print('All dependencies installed successfully!')"
```

**Step 3: Test Installation**
```bash
# Quick test run
python demos/satellite_visualization.py
```

If you see a space scene with a rotating satellite, your installation is successful!

### First Run

**Launch the main demonstration:**
```bash
python demos/satellite_visualization.py
```

**What you should see:**
- A dark space background with twinkling stars
- Earth in the lower left with rotating surface features
- A detailed satellite in the center with solar panels
- Real-time telemetry displays showing orientation data
- Performance graphs tracking error and control effort

---

## Running the Demonstrations

### Main Visualization Demo

**File:** `demos/satellite_visualization.py`

**Description:** The primary interactive demonstration featuring advanced graphics, realistic physics, and comprehensive telemetry displays.

**Launch Command:**
```bash
python demos/satellite_visualization.py
```

**Features:**
- Real-time 3D satellite visualization
- Detailed Earth rendering with atmospheric effects
- Particle effects for thruster activation
- Performance monitoring graphs
- Interactive control interfaces

### Educational Development Files

**Mathematical Analysis** (`archive/main.py`):
```bash
python archive/main.py
```

This script demonstrates:
- 3D rotation mathematics fundamentals
- Gimbal lock analysis and detection
- Rotation matrix operations
- Euler angle conversions

**PID Controller Analysis** (`archive/pid.py`):
```bash
python archive/pid.py
```

This script shows:
- PID controller implementation
- Real-time attitude control simulation
- Performance analysis and metrics
- Step response characteristics

### Jupyter Notebook Tutorials

**Interactive Learning Notebook:**
```bash
jupyter notebook documentation/satellite_attitude_control_tutorial_en.ipynb
```

**Features:**
- Step-by-step mathematical derivations
- Interactive code cells with immediate feedback
- Comprehensive visualizations and explanations
- Progressive learning from basics to advanced concepts

---

## Understanding the Interface

### Main Display Elements

**🌌 Space Environment**
- **Starfield**: Realistic star distribution with twinkling effects
- **Earth**: Rotating planet with continental features and atmospheric glow
- **Background**: Deep space gradient creating immersive environment

**🛰️ Spacecraft Visualization**
- **Main Body**: Central satellite structure with realistic proportions
- **Solar Panels**: Extended blue panels that rotate with spacecraft
- **Attitude Vectors**: Colored arrows showing spacecraft orientation
  - **Red Arrow**: X-axis (Roll direction)
  - **Green Arrow**: Y-axis (Pitch direction)  
  - **Blue Arrow**: Z-axis (Yaw direction)

**📊 Telemetry Displays**

*Left Panel - Current Status:*
```
Attitude Control System
Current Orientation:
  Roll:   +15.32°
  Pitch:  -8.47°
  Yaw:    +22.15°
Target Orientation:
  Roll:    0.00°
  Pitch:   0.00°
  Yaw:     0.00°
```

*Right Panel - Performance Graphs:*
- **Orientation Error**: Real-time tracking of pointing accuracy
- **Control Effort**: Magnitude of control torques being applied
- **System Status**: Current operational state (STABLE/CONVERGING/CORRECTING)

### Visual Indicators

**🟢 STABLE** - Error < 1° (Green status)
- System has converged to target orientation
- Minimal control effort required
- Ready for precision operations

**🟡 CONVERGING** - Error 1-5° (Yellow status)
- System actively reducing error
- Moderate control effort
- Approaching target orientation

**🔴 CORRECTING** - Error > 5° (Red status)
- Large orientation error present
- High control effort
- Initial acquisition or disturbance recovery

---

## Interactive Controls

### Keyboard Commands

**Primary Controls:**
- **SPACE**: Apply random disturbance (tests controller robustness)
- **R**: Reset simulation to initial conditions
- **T**: Set new random target orientation
- **ESC**: Exit simulation

**Advanced Controls:**
- **1-9**: Adjust PID gain values in real-time
- **Arrow Keys**: Manual spacecraft rotation (override mode)
- **P**: Pause/Resume simulation
- **S**: Take screenshot
- **H**: Show/hide help overlay

### Mouse Interactions

**Camera Control:**
- **Left Click + Drag**: Rotate view around spacecraft
- **Right Click + Drag**: Pan view
- **Scroll Wheel**: Zoom in/out
- **Middle Click**: Reset camera to default position

### Real-Time Parameter Adjustment

**PID Tuning Interface:**
```
Current PID Gains:
  Kp (Proportional): 3.0  [Press 1/2 to adjust]
  Ki (Integral):     0.2  [Press 3/4 to adjust]
  Kd (Derivative):   0.8  [Press 5/6 to adjust]
```

**Effect of Parameter Changes:**
- **Increasing Kp**: Faster response, potential overshoot
- **Increasing Ki**: Eliminates steady-state error, may cause oscillation
- **Increasing Kd**: Reduces overshoot, improves stability

---

## Customizing Simulations

### Configuration Files

**Create custom scenarios** by modifying initial conditions:

```python
# Custom initial configuration
initial_conditions = {
    'roll': 45.0,      # degrees
    'pitch': 30.0,     # degrees  
    'yaw': -60.0,      # degrees
    'target_roll': 0.0,
    'target_pitch': 0.0,
    'target_yaw': 0.0
}

# Custom PID parameters
pid_gains = {
    'kp': 3.0,  # Proportional gain
    'ki': 0.2,  # Integral gain
    'kd': 0.8   # Derivative gain
}
```

### Scenario Examples

**1. Station-Keeping Scenario:**
```python
# Maintain precise Earth-pointing attitude
scenario = {
    'name': 'Earth Observation',
    'initial_attitude': [2.0, 1.5, 0.8],
    'target_attitude': [0.0, 0.0, 0.0],
    'disturbance_level': 'low',
    'precision_requirement': 0.1  # degrees
}
```

**2. Slew Maneuver Scenario:**
```python
# Large attitude change testing
scenario = {
    'name': 'Rapid Reorientation',
    'initial_attitude': [90.0, 45.0, 180.0],
    'target_attitude': [0.0, 0.0, 0.0],
    'disturbance_level': 'medium',
    'time_constraint': 30.0  # seconds
}
```

**3. Disturbance Rejection Scenario:**
```python
# Test controller robustness
scenario = {
    'name': 'Solar Storm Simulation',
    'initial_attitude': [0.0, 0.0, 0.0],
    'target_attitude': [0.0, 0.0, 0.0],
    'disturbance_level': 'high',
    'disturbance_frequency': 5.0  # seconds
}
```

### Advanced Customization

**Custom Physics Parameters:**
```python
# Modify spacecraft properties
spacecraft_config = {
    'inertia': 1.5,        # Moment of inertia
    'damping': 0.03,       # Environmental damping
    'max_torque': 10.0,    # Actuator limits
    'sensor_noise': 0.1    # Measurement uncertainty
}
```

---

## Educational Applications

### Curriculum Integration

**Undergraduate Courses:**
- **Controls Engineering**: PID controller design and tuning
- **Aerospace Engineering**: Spacecraft dynamics and attitude control
- **Applied Mathematics**: 3D rotations and matrix operations
- **Physics**: Classical mechanics and rotational dynamics

**Graduate Research:**
- Advanced control algorithms (LQR, MPC, adaptive control)
- Quaternion-based attitude representation
- Multi-satellite formation flying
- Fault-tolerant control systems

### Laboratory Exercises

**Exercise 1: PID Tuning Workshop**
```
Objective: Optimize controller performance for different scenarios
Duration: 2 hours
Materials: Simulation software, parameter sheets
Procedure:
1. Start with default PID gains
2. Apply step input and record response
3. Adjust parameters systematically
4. Document performance metrics
5. Compare different tuning methods
```

**Exercise 2: Gimbal Lock Investigation**
```
Objective: Understand Euler angle singularities
Duration: 1.5 hours
Materials: Mathematical analysis scripts
Procedure:
1. Run gimbal lock analysis (archive/main.py)
2. Observe rotation matrix behavior near ±90° pitch
3. Compare with quaternion representation
4. Document findings and implications
```

**Exercise 3: Disturbance Analysis**
```
Objective: Evaluate controller robustness
Duration: 2.5 hours
Materials: Full simulation environment
Procedure:
1. Establish baseline performance
2. Apply various disturbance levels
3. Measure recovery time and overshoot
4. Optimize controller for robustness
5. Present findings in engineering report
```

### Assessment Methods

**Performance Metrics:**
- **Settling Time**: Time to reach ±1° of target
- **Overshoot**: Maximum error during response
- **Steady-State Error**: Final pointing accuracy
- **Control Effort**: Energy consumption metrics

**Rubric Example:**
```
Excellent (90-100%):
- Achieves < 0.5° steady-state error
- Settling time < 10 seconds
- Minimal overshoot (< 5°)
- Justifies design decisions clearly

Good (80-89%):
- Achieves < 1.0° steady-state error
- Settling time < 15 seconds
- Moderate overshoot (< 10°)
- Explains most design choices

Satisfactory (70-79%):
- Achieves < 2.0° steady-state error
- Settling time < 25 seconds
- Some overshoot present
- Basic understanding demonstrated
```

---

## Advanced Features

### Multi-Scenario Analysis

**Batch Processing:**
```python
# Run multiple scenarios automatically
scenarios = [
    {'name': 'Low_Disturbance', 'disturbance': 0.1},
    {'name': 'Medium_Disturbance', 'disturbance': 1.0},
    {'name': 'High_Disturbance', 'disturbance': 5.0}
]

for scenario in scenarios:
    run_simulation(scenario)
    save_results(scenario['name'])
    generate_report(scenario['name'])
```

### Data Export and Analysis

**Performance Data Export:**
```python
# Export simulation data for external analysis
export_config = {
    'format': 'csv',        # csv, json, matlab
    'variables': ['time', 'error', 'control_effort', 'angles'],
    'sampling_rate': 10,    # Hz
    'filename': 'simulation_data.csv'
}
```

**Integration with Analysis Tools:**
- **MATLAB**: Import simulation data for advanced analysis
- **Python**: Use pandas, scipy for statistical analysis
- **R**: Statistical modeling and visualization
- **Excel**: Basic performance tracking and reporting

### Hardware-in-the-Loop (HIL) Preparation

**Code Structure for HIL:**
```python
class AttitudeController:
    def __init__(self, hardware_interface=None):
        self.hardware = hardware_interface
        
    def get_sensor_data(self):
        if self.hardware:
            return self.hardware.read_sensors()
        else:
            return self.simulated_sensors()
    
    def send_control_commands(self, torques):
        if self.hardware:
            self.hardware.send_torques(torques)
        else:
            self.simulate_torques(torques)
```

---

## Troubleshooting

### Common Issues and Solutions

**Problem**: Simulation runs slowly or choppy
**Solutions:**
- Reduce window size or disable advanced graphics
- Close other applications to free system resources
- Update graphics drivers
- Use dedicated graphics card if available

**Problem**: Import errors on startup
**Solutions:**
```bash
# Reinstall dependencies
pip install --upgrade -r requirements.txt

# Check Python version
python --version  # Should be 3.8+

# Verify specific packages
python -c "import numpy; print(numpy.__version__)"
python -c "import pygame; print(pygame.version.ver)"
```

**Problem**: Controller doesn't stabilize
**Solutions:**
- Check PID gains aren't too aggressive
- Verify initial conditions aren't extreme
- Reset simulation (press 'R')
- Try conservative PID values: kp=1.0, ki=0.1, kd=0.2

**Problem**: Graphics display issues
**Solutions:**
- Update graphics drivers
- Try different display modes:
```python
# Add to visualization script
pygame.display.set_mode((width, height), pygame.DOUBLEBUF)
```

### Debug Mode

**Enable detailed logging:**
```python
# Add to beginning of simulation script
import logging
logging.basicConfig(level=logging.DEBUG)

# This will show detailed execution information
```

**Performance Monitoring:**
```python
# Add frame rate display
fps_clock = pygame.time.Clock()
fps = fps_clock.get_fps()
print(f"Current FPS: {fps:.1f}")
```

---

## Best Practices

### Simulation Design

**Start Simple:**
1. Begin with basic scenarios
2. Use default PID parameters
3. Understand baseline behavior
4. Gradually increase complexity

**Systematic Approach:**
1. Define clear objectives
2. Document initial conditions
3. Record all parameter changes
4. Analyze results methodically

### Educational Use

**Preparation:**
- Test all demonstrations before class
- Prepare backup scenarios
- Have troubleshooting guide ready
- Practice explaining key concepts

**Engagement:**
- Encourage hands-on experimentation
- Ask predictive questions before changes
- Connect to real-world applications
- Relate to current space missions

### Performance Optimization

**For Smooth Operation:**
- Close unnecessary applications
- Use fullscreen mode for best performance
- Adjust graphics quality based on hardware
- Monitor system resources during operation

**For Accurate Results:**
- Use consistent time steps
- Minimize external disturbances during data collection
- Repeat experiments for statistical validity
- Document environmental conditions

### Safety and Reliability

**Data Backup:**
- Save important simulation results immediately
- Use version control for custom modifications
- Document all changes and their effects
- Maintain clean baseline configurations

**System Maintenance:**
- Regularly update dependencies
- Clear temporary files periodically
- Monitor disk space usage
- Keep system drivers updated

---

## Conclusion

This spacecraft attitude control simulation provides a comprehensive platform for education, research, and algorithm development. By following this user guide, you can effectively utilize all features of the system while avoiding common pitfalls.

For additional support, advanced features, or custom development needs, please refer to the technical documentation in the `docs/` folder or contact the development team.

**Happy simulating! 🚀**
