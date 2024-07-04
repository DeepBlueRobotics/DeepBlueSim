This project tests DeepBlueSim. It uses the DeepBlueSim plugin and contains tests that refer to
Webots worlds in `../plugin/controller/src/webotsFolder/dist/worlds`. That makes it easier to debug
those worlds and the various DeepBlueSim Webots PROTOs. Note: the contents of *this* project's
`Webots` folder are ignored.

By convention, the test classes and robot classes are named after the Webots worlds. For example,
`DBSExample.java` contains the robot code for controlling the Webots robot in `DBSExample.wbt` and
`DBSExampleTest.java` contains tests that verify that is working correctly. If, instead of running
the tests, you want to simulate one of the robots using VSCode's `WPILib: Simulate Robot Code`, you
need to first change the robot provider passed to `RobotBase.start()` in `Main.java`.
