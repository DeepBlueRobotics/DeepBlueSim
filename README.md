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

 1. Install the latest [official nightly build of Webots](https://github.com/cyberbotics/webots/releases).
 1. Add the DeepBlueSim Gradle plugin to your `build.gradle` by adding the following line
 in your `plugins` section:
 ```
     id "org.carlmontrobotics.deepbluesim" version "0.0.15"
 ```

### Demo

 1. In VSCode run the `WPILib: Create a new project` command and create the example project.
 2. Add the DeepBlueSim Gradle plugin to the `build.gradle` as described above.
 3. If you are not using [`lib199`](https://github.com/DeepBlueRobotics/lib199), add the following 2
    lines to your robot's `simulationInit()` method (`lib199` handles this automatically):
 ```java
     @Override
    public void simulationInit() {
        // Regularly request a HALSimWS connection from the DeepBlueSim controller (if/when it is
        // listening). To workaround https://github.com/wpilibsuite/allwpilib/issues/6842, this must
        // be done *after* any SimDevices have been created.
        var reqPublisher = NetworkTableInstance.getDefault()
                .getStringTopic("/DeepBlueSim/Coordinator/request").publish();
        addPeriodic(() -> reqPublisher.set("connectHALSimWS"), kDefaultPeriod);
    }
 ```
 4. In VSCode run the `WPILib: Simulate Robot Code on Desktop` command and select both
 `libhalsim_gui` and `libhalsim_ws_server` as the extensions to use.
 5. Start Webots and open *your_example_project*`/Webots/worlds/DBSExample.wbt`
 6. In the HALSim GUI, select `Autonomous` to see the robot drive forward for 2 seconds, or
 select `Teleop` and use the keyboard on joystick to drive the robot around.

## Details

## Command Line Options
The controller provides some command line options that can be used to configure
different features. These can be specified through the `controllerArgs` field in
Webots or on the command line (if run as an external controller). They are primarily
intended for debugging. For the majority of cases, the default settings should be sufficient.

At the moment, the following options are supported:
```
--no-network-tables   Do not attempt to connect to Network Tables
--no-robot-code       Do not attempt to connect to the HALSim Server
```

### Time synchronization and writing tests

By default, the robot code and Webots run at their own speeds so their clocks will not necessarily
match. This can be particularly problematic when trying to write tests which should be as
deterministic as possible. To synchronize the clocks, tests should use the
`org.carlmontrobotics.libdeepbluesim.WebotsSimulator` class as demonstrated in the example's
[`SystemTestRobot.java`](example/src/systemTest/java/frc/robot/SystemTestRobot.java). In addition to
synchronizing the clocks, the `WebotsSimulator` class provides methods for accessing the state of
the simulation at every time step and at specific times, as well as methods to enable the robot in
autonomous and teleop.

## Contributing

NOTE: If you are using VSCode and attempting to work on the code in
`plugin/controller/`, VSCode may complain about being unable to resolve various
symbols from `WPIWebSockets`. This is because VSCode's Language Support for Java
doesn't currently handle dependencies in Gradle "included builds". The workaround
is to run `gradlew :WPIWebSockets:publishToMavenLocal` and then tell VSCode to
"Java: Clean Language Server Workspace".

For debugging, the controller can be run externally. To do this, set the robot's
controller in Webots to `<extern>`, and in VSCode go to
`Run and Debug > Launch Extern Controller` or press `F5`. This will launch the
controller and connect it to VSCode's debugger. There is also a
`Launch Extern Controller (No Robot Code)` configuration which passes the
`--no-robot-code` argument to the controller so that controller-specific behavior
can be tested.

In addition to these features, there is an `Update example/Webots directory` task
which will build the plugin and install it to the `example/Webots` directory
without running additional tests. This is helpful for debugging Webots specific
features (such as PROTOs). There is also a `(Watch)` variant which scans for
changes in the plugin code and updates the directory automatically. Note that
any time the directory is updated, `DBSExample.wbt` is overwritten. The recommended
way to work around this is to keep Webots open while the directory is reloaded and
then Save and Reload the world to preserve any changes to the world file.
