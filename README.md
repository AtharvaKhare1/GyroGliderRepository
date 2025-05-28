# Gyroglider

A self-balancing robotic platform using BLDC motors and PID control. This research project was developed under the Eklavya Mentorship Program at Society of Robotics and Automation (SRA), VJTI Mumbai.

## 🎯 Project Overview

The Gyroglider is a research project focused on developing a self-balancing system that can maintain stability at specific angles using real-time control algorithms. The system combines mechanical engineering with control theory to create a platform suitable for various balancing and stabilization applications.

### Key Features
- **Self-Balancing Control**: Maintains balance using PID control algorithm
- **DSHOT Protocol**: Digital communication for precise motor control
- **Real-time Corrections**: Dynamic throttle adjustments for stability
- **Custom Chassis Design**: Engineered aluminum T-shaped structure
- **6-axis Motion Sensing**: MPU6050 for motion tracking and feedback

### Applications
- Dynamic balancing systems
- Stabilization platforms
- Control system research
- Educational robotics projects
- Industrial automation prototypes

## 🏗️ System Architecture

### Hardware Components
- **ESP32 Wrover SoC**: Dual-core processing unit with integrated WiFi/Bluetooth
- **Custom SRA Development Board**: Industrial-grade I/O with protection circuits
- **DYS 60A ARIA 4-in-1 ESC**: Professional-grade electronic speed controller
- **MPU6050 IMU**: 6-axis inertial measurement unit with DMP processing
- **Emax 2000KV BLDC Motors** (2x): High-performance brushless propulsion system
- **Precision-Engineered Chassis**: FEA-optimized aluminum construction
- **High-Capacity Power System**: 24V LiPo battery with protection circuitry

### Software Stack
- **Real-time Operating System**: ESP-IDF with FreeRTOS kernel
- **Embedded Systems Programming**: Optimized C implementation
- **Advanced Control Algorithms**: Multi-parameter PID with adaptive tuning
- **Digital Communication Stack**: DSHOT protocol implementation
- **IoT Connectivity**: WebSocket server for industrial monitoring
- **Sensor Fusion Algorithms**: Kalman filtering for enhanced accuracy

## 🔬 Development Process

### Design & Analysis
- **3D Modeling**: Chassis design using Fusion 360
- **Structural Analysis**: Component validation using ANSYS
- **Control System Design**: PID implementation and tuning
- **Protocol Implementation**: DSHOT digital communication

### Hardware Integration
- **Component Assembly**: Precision mounting and wiring
- **System Testing**: Performance validation and debugging
- **Safety Implementation**: Fail-safe mechanisms and monitoring
- **Parameter Tuning**: PID optimization for stable operation

## 🚀 Getting Started

### Prerequisites
- ESP-IDF v4.4+ development environment
- Fusion 360 Professional (for design modifications)
- ANSYS Workbench (for structural analysis)
- Advanced embedded systems knowledge
- Control systems theory background

### Hardware Assembly
1. **Chassis Preparation**
   - Laser cut the T-shaped aluminum base plate
   - Drill mounting holes for components

2. **Component Mounting**
   - Mount BLDC motors at horizontal ends
   - Install MPU6050 at center of mass with spacers
   - Mount SRA board and ESC with proper isolation
   - Connect hinge mechanism for stability

3. **System Integration**
   - Connect ESC to BLDC motors
   - Wire MPU6050 via I2C communication
   - Set up power distribution from battery
   - Implement safety measures and grounding

### Software Installation
```bash
# Clone the repository
git clone https://github.com/AtharvaKhare1/GyroGliderRepository.git
cd GyroGliderRepository

# Set up ESP-IDF environment
. $HOME/esp/esp-idf/export.sh

# Build and flash
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## 📊 Technical Specifications

### DSHOT Communication Protocol
| Protocol | Bitrate | TH1 (μs) | TH0 (μs) | Bit Time (μs) | Frame Time (μs) |
|----------|---------|----------|----------|---------------|-----------------|
| DSHOT150 | 150kbit/s | 5.00 | 2.50 | 6.67 | 106.72 |
| DSHOT300 | 300kbit/s | 2.50 | 1.25 | 3.33 | 53.28 |
| DSHOT600 | 600kbit/s | 1.25 | 0.625 | 1.67 | 26.72 |

### PID Control System
- **Proportional (P)**: Responds to current error magnitude
- **Integral (I)**: Eliminates steady-state errors over time
- **Derivative (D)**: Provides damping based on error rate of change

## 🔧 Configuration

### PID Tuning Parameters
```c
// PID parameters (adjust based on your system)
float kp = 1.0;  // Proportional gain
float ki = 0.1;  // Integral gain  
float kd = 0.05; // Derivative gain
```

### Motor Configuration
- Motor KV Rating: 2000KV
- ESC Current Rating: 60A per channel
- Operating Voltage: 24V LiPo

## 📈 Project Status

### Completed Features
- ✅ PID control system implementation
- ✅ DSHOT protocol integration
- ✅ 3D modeling and structural analysis
- ✅ Hardware assembly and testing
- ✅ Embedded C programming

### Current Status
- System demonstrates balancing capabilities
- Digital communication protocols working
- Hardware integration complete
- Further optimization possible for enhanced performance

## 🔮 Future Development

- Enhanced PID tuning algorithms
- Improved mechanical design for better stability
- Integration of additional sensors
- Autonomous navigation capabilities
- Advanced control system implementations

## 👥 Team

- **Soham Kute** 
- **Atharva Khare** 
- **Ameya Tikhe** 
- **Kesar Sutar** 

### Mentors
- **Shankari Anandakrishnan**
- **Atharva Kashalkar**

## 🏛️ Institution

**Society of Robotics and Automation (SRA)**  
Veermata Jijabai Technological Institute (VJTI)  
Mumbai - 400019

## 📚 References

- Linear Algebra fundamentals
- ESP-IDF Framework documentation
- Understanding PID Control systems
- DSHOT protocol specifications

## 📄 License

This project is developed as part of the Eklavya Mentorship Program. Please contact the team for usage permissions.

## 🤝 Contributing

We welcome contributions to improve the Gyroglider project! Please feel free to:
- Report bugs and issues
- Suggest improvements
- Submit pull requests
- Share your experience with the project

---

*For more information about the Society of Robotics and Automation at VJTI, visit our official channels.*
