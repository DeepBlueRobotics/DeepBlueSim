**NOTE: This project has the `./example` project's system test as a dependency of the `check` (and
thus indirectly the `build`) Gradle task. For the system test to pass, Webots must be running the
`Webots/worlds/DBSExample.wbt` world. If it isn't, the test will wait for up to 30 minutes for it to
be started before failing with a timeout error.**
