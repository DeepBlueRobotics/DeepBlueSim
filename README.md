# DeepBlueSim

Simulate [FIRST Robotics Competition (FRC)](https://www.firstinspires.org/robotics/frc) robots
in the [Webots](https://cyberbotics.com/) robot simulator.

## Status

We (Team 199, Deep Blue) are actively developing it. It is working for us,
but **the implementation is still changing rapidly**.

See the issues for likely changes in the near future.

Works on Windows, MacOS, and Linux.

Works with Java WPILib projects and (probably) C++ WPILib projects because it takes
advantage of the WPILib's WebSockets server desktop simulation extension.

## Quick Start

 1. Add the Gradle plugin to your `build.gradle` by adding the following line
 in your `plugins` section:
 ```
     id "org.team199.deepbluesim" version "0.0.1"
 ```
 2. Run the `installDeepBlueSim` Gradle task. This will create a `Webots` folder at the
 top-level of your project.
 3. Copy the `./example/Webots/worlds/Test.wbt` file to `*your_project*/Webots/worlds/Test.wbt`
 4. Install Webots if you don't already have it installed.
 5. Start Webots and open `*your_project*/Webots/worlds/Test.wbt`
 6. Within VSCode run the `WPILib: Simulate Robot Code on Desktop` command and select both
 `libhasim_gui` and `libhalsim_ws_server` as the extensions to use.
 7. In the HALSim GUI, select `Autonomous` to see the robot drive forward for 2 seconds, or
 select `Teleop` and use the keyboard on joystick to drive the robot around.



