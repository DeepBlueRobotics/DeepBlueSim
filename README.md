# DeepBlueSim

Simulate [FIRST Robotics Competition (FRC)](https://www.firstinspires.org/robotics/frc) robots
in the [Webots](https://cyberbotics.com/) robot simulator.

## Status

![CI](https://github.com/DeepBlueRobotics/DeepBlueSim/workflows/CI/badge.svg)

We ([Team 199, Deep Blue](http://www.carlmontrobotics.org)) are actively developing it. 
It is working for us, but **the implementation is still changing rapidly**.

See the [issues](https://github.com/DeepBlueRobotics/DeepBlueSim/issues) for likely changes 
in the near future.

Works on Windows, MacOS, and Linux.

Works with Java WPILib projects and (probably) C++ WPILib projects because it takes
advantage of the WPILib's WebSockets server desktop simulation extension.

## Quick Start

### Installation

 1. Install Webots if you don't already have it installed.
 1. Add the DeepBlueSim Gradle plugin to your `build.gradle` by adding the following line
 in your `plugins` section:
 ```
     id "org.team199.deepbluesim" version "0.0.12"
 ```

### Demo

 1. In VSCode run the `WPILib: Create a new project` command and create the example project.
 1. Add the DeepBlueSim Gradle plugin to the `build.gradle` as described above.
 1. In VSCode run the `WPILib: Simulate Robot Code on Desktop` command and select both
 `libhalsim_gui` and `libhalsim_ws_server` as the extensions to use.
 1. Start Webots and open *your_example_project*`/Webots/worlds/DBSExample.wbt`
 1. In the HALSim GUI, select `Autonomous` to see the robot drive forward for 2 seconds, or
 select `Teleop` and use the keyboard on joystick to drive the robot around.



