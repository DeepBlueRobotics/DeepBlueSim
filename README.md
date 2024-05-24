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
     id "org.team199.deepbluesim" version "0.0.15"
 ```

### Demo

 1. In VSCode run the `WPILib: Create a new project` command and create the example project.
 1. Add the DeepBlueSim Gradle plugin to the `build.gradle` as described above.
 1. In VSCode run the `WPILib: Simulate Robot Code on Desktop` command and select both
 `libhalsim_gui` and `libhalsim_ws_server` as the extensions to use.
 1. Start Webots and open *your_example_project*`/Webots/worlds/DBSExample.wbt`
 1. In the HALSim GUI, select `Autonomous` to see the robot drive forward for 2 seconds, or
 select `Teleop` and use the keyboard on joystick to drive the robot around.

## Details

### Time synchronization

By default, the robot code and Webots run at their own speeds so their clocks
will not necessarily match. This can be particularly problematic when trying to
write tests which should be as deterministic as possible. To synchronize the
clocks, the robot code can create a `SimDeviceSim` named `TimeSynchronizer` with
2 `double` values: `robotTimeSec` (output direction) and `simTimeSec` (input
direction). To start the synchronization and force Webots to reload the world
(so that it is in a known state with time = 0), the robot code should set
`robotTimeSec` to -2. The simulator will respond by setting `simTimeSec` to -2.
At that point the robot code can set `robotTimeSec` to the current robot time
and the simulator will run until it's time is greater than that. As it runs, it
will also updae `simTimeSec`. The robot code can ensure that it doesn't get
ahead of the simulator by using `SimHooks.pauseTiming()` to pause the robot code
when the robot time is ahead of the simulation time and
`SimHooks.resumeTiming()` when it is not. See the `webotsInit()` method of the
example's
[`SystemTestRobot.java`](example/src/systemTest/java/frc/robot/SystemTestRobot.java)
for an example implementation.

## Contributing

NOTE: If you are using VSCode and attempting to work on the code in
`plugin/controller/`, VSCode may complain about being unable to resolve various
symbols from `WPIWebSockets`. This is because VSCode's Language Support for Java
doesn't currently handle dependencies in Gradle "included builds". The workaround
is to run `gradlew :WPIWebSockets:publishToMavenLocal` and then tell VSCode to
"Java: Clean Language Server Workspace".
