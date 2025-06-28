# Pneumatic Actuator Simulation & Analysis Tool

A comprehensive C++23 simulation system for analyzing pneumatic actuator dynamics with real-time visualization and statistical analysis.

## Features

### 🎛️ Parameter Control Module
- **Cylinder Parameters**: Piston diameter, rod diameter, stroke length
- **Gas Parameters**: Inlet/outlet pressure settings
- **Load Parameters**: Mass configuration
- **Initial Conditions**: Position and velocity settings
- **Time Parameters**: Simulation start/end time
- **Solver Parameters**: Step size and convergence settings
- **Real-time Status**: Live calculation progress tracking

### 📊 Visualization Module
- **Position vs Time Plot**: y(t) - Track actuator position over time
- **Velocity vs Time Plot**: v(t) - Monitor velocity dynamics
- **Phase Portrait**: v(x) - Velocity vs position relationship
- **Interactive Controls**: Grid, legend, auto-fit options
- **Start/End Markers**: Clear visualization of trajectory endpoints

### 📈 Statistics Module
- **Timing Information**: Calculation time and performance metrics
- **Solver Statistics**: Steps taken, convergence status, utilization
- **Result Summary**: Maximum values, final state, simulation metrics
- **System Information**: Parameter display and calculated values
- **Real-time Updates**: Automatic refresh with configurable intervals

## Architecture

### Modern C++23 Design
- **RAII Principles**: Automatic resource management
- **Concepts**: Type-safe template constraints
- **Ranges**: Modern iteration and algorithms
- **Smart Pointers**: Memory-safe object management
- **Atomic Operations**: Thread-safe state management

### Modular Architecture
```
┌─────────────────────┐
│   Application       │
├─────────────────────┤
│ ┌─────────────────┐ │
│ │ Parameter Module│ │
│ └─────────────────┘ │
│ ┌─────────────────┐ │
│ │  Chart Module   │ │
│ └─────────────────┘ │
│ ┌─────────────────┐ │
│ │Statistics Module│ │
│ └─────────────────┘ │
└─────────────────────┘
         │
┌─────────────────────┐
│  Pneumatic Model    │
├─────────────────────┤
│ • ODE Solver        │
│ • Physics Engine    │
│ • Result Storage    │
│ • Statistics        │
└─────────────────────┘
```

### Dependencies
- **Eigen3**: Linear algebra and numerical computations
- **ImGui**: Immediate mode GUI framework
- **ImPlot**: Scientific plotting library
- **GLFW**: Cross-platform windowing and input
- **GLAD**: OpenGL loading library

## Physics Model

The system simulates a pneumatic actuator using a second-order differential equation:

```
d²x/dt² = (P_in × A_effective - P_out × A_rod) / m
```

Where:
- `P_in`, `P_out`: Inlet and outlet pressures
- `A_effective`: Effective piston area
- `A_rod`: Rod area
- `m`: Load mass

### Numerical Integration
- **Explicit Euler Method**: Fast, conditionally stable
- **Configurable Step Size**: Balance between accuracy and performance
- **Thread-safe Execution**: Non-blocking UI during calculations

## User Interface

### Professional Layout
- **Dockable Windows**: Flexible workspace organization
- **Tooltips**: Comprehensive parameter descriptions
- **Color-coded Status**: Visual feedback for calculation state
- **Responsive Design**: Adapts to different window sizes

### Workflow
1. **Configure Parameters**: Set cylinder, gas, and load parameters
2. **Set Initial Conditions**: Define starting position and velocity
3. **Configure Solver**: Choose step size and limits
4. **Run Simulation**: Click "Start Calculation"
5. **Analyze Results**: View charts and statistics

## Build Instructions

### Prerequisites
- Visual Studio 2022 (Windows)
- CMake 3.20+
- vcpkg package manager

### Building
```bash
# Install dependencies
.\.vcpkg\vcpkg.exe install --recurse

# Configure project
cmake -B build -DCMAKE_TOOLCHAIN_FILE=".vcpkg/scripts/buildsystems/vcpkg.cmake"

# Build
cmake --build build --config Release

# Run
.\build\Release\PneumaticActuator.exe
```

## Key Implementation Highlights

### Type Safety
- Fixed linter errors with proper type conversions (double ↔ float)
- Used concepts for template constraints
- Employed `[[nodiscard]]` for important return values

### Performance Optimization
- Cached plot data to avoid constant memory allocation
- Thread-safe result access with mutex protection
- Efficient hash-based change detection for UI updates

### Error Handling
- Comprehensive exception handling throughout
- Graceful degradation when data is unavailable
- Detailed logging with structured messages

### Memory Management
- RAII-based resource management
- Smart pointers for automatic cleanup
- Reserved containers to minimize allocations

## Future Enhancements

- **Multiple Solver Types**: Implicit methods, Runge-Kutta
- **Advanced Physics**: Friction, air resistance, temperature effects
- **Data Export**: CSV, JSON result export capabilities
- **Parameter Sweeps**: Batch analysis with parameter variations
- **3D Visualization**: Real-time 3D actuator animation

## License

This project demonstrates modern C++ engineering practices and scientific computing techniques for pneumatic system simulation. 