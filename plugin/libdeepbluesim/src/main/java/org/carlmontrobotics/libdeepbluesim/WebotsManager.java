package org.carlmontrobotics.libdeepbluesim;

import java.util.EnumSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.function.Consumer;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;

/**
 * A manager for Webots robots that use the DeepBlueSim controller. It keeps the time in the
 * simulator synchronized to the robot time and provides a way to add Watchers to track the
 * kinematics of Webots nodes.
 */
public class WebotsManager implements AutoCloseable {
    private final TimedRobot robot;
    private final Timer robotTime = new Timer();
    private final NetworkTableInstance inst;
    private final NetworkTable coordinator;
    private final StringEntry reloadStatusEntry;
    private final DoublePublisher robotTimeSecPublisher;
    private final DoubleSubscriber simTimeSecSubscriber;

    // Use these to control NetworkTables logging.
    // - ntLoglevel = 0 means no NT logging
    // - ntLogLevel > 0 means log NT log messages that have a level that is >= *both* ntLogLevel and
    // ntTransientLogLevel. Typically set ntLogLevel = LogMessage.kDebug4 and then, while running
    // code requiring detailed logging, set ntTransientLogLevel to LogMessage.kDebug4.
    private static int ntLogLevel = 0;
    private static volatile int ntTransientLogLevel = LogMessage.kInfo;

    @SuppressWarnings("resource")
    private final Notifier pauser = new Notifier(() -> {
        // This is replaced in waitForUserToStart()
    });

    /**
     * Constructs an instance that connects the robot code to the Webots-simulated robot controlled
     * by DeepBlueSim.
     * 
     * @param robot the robot to connect to the Webots-simulated robot.
     */
    public WebotsManager(TimedRobot robot) {
        this.robot = robot;
        inst = NetworkTableInstance.getDefault();
        if (ntLogLevel > 0)
            inst.addLogger(ntLogLevel, Integer.MAX_VALUE, (event) -> {
                if (event.logMessage.level < ntTransientLogLevel)
                    return;
                System.out
                        .println("NT instance log level %d message: %s(%d): %s"
                                .formatted(event.logMessage.level,
                                        event.logMessage.filename,
                                        event.logMessage.line,
                                        event.logMessage.message));
            });

        coordinator = inst.getTable("/DeepBlueSim/Coordinator");
        var pubSubOptions = new PubSubOption[] {
                PubSubOption.sendAll(true),
                PubSubOption.periodic(Double.MIN_VALUE)
        };
        var reloadStatusTopic = coordinator.getStringTopic("reloadStatus");
        reloadStatusTopic.setCached(false);
        reloadStatusEntry =
                reloadStatusTopic
                .getEntry("", PubSubOption.sendAll(true));
        robotTimeSecPublisher = coordinator.getDoubleTopic("robotTimeSec")
                .publish(pubSubOptions);
        simTimeSecSubscriber = coordinator.getDoubleTopic("simTimeSec")
                .subscribe(-1.0, pubSubOptions);
    }

    /**
     * Gets the current robot time.
     * 
     * @return the current robot time in seconds.
     */
    public double getTimeSecs() {
        return robotTime.get();
    }

    /**
     * Adds a Watcher for a Webots node.
     * 
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     * @return the Watcher object.
     */
    public Watcher addWatcher(String defPath) {
        return new Watcher(defPath);
    }

    /**
     * Asks the user to load and start a particular Webots world and waits until they have done so.
     * 
     * @param worldFile the world file's path to be displayed to the user.
     * @throws TimeoutException if the world hasn't been started in time.
     * 
     * @return this object for chaining
     */
    public WebotsManager waitForUserToStart(String worldFile)
            throws TimeoutException {
        // Tell the user to load the world file.
        String userReminder =
                "Waiting for Webots to be ready. Please open %s in Webots."
                        .formatted(worldFile);
        System.err.println(userReminder);

        // Start robot time right before the first periodic call is made so that we ignore
        // startup time.
        robotTime.stop();
        robotTime.reset();
        // Set the offset to -period to run before other WPILib periodic methods
        robot.addPeriodic(robotTime::start, robot.getPeriod(),
                -robot.getPeriod());

        // Reset the clock. Without this, *Periodic calls that should have
        // occurred while we waited, will be considered behind schedule and
        // will all happen at once.
        SimHooks.restartTiming();

        // Pause the clock so that we can step it in sync with the simulator
        SimHooks.pauseTiming();

        final var isReadyFuture = new CompletableFuture<Boolean>();

        // When DeepBlueSim says that reload has completed, we are ready
        inst.addListener(reloadStatusEntry.getTopic(),
                EnumSet.of(Kind.kValueRemote, Kind.kImmediate), (event) -> {
                    final String reloadStatus =
                            event.valueData.value.getString();
                    System.out.println(
                            "In listener, reloadStatus = %s"
                                    .formatted(reloadStatus));
                    if (!reloadStatus.equals("Completed"))
                        return;
                    isReadyFuture.complete(true);
                });

        // If DeepBlueSim is already running or when the user starts it, tell it to reload the world
        // so it is in a known state. It will set this value to "In Progress" before it reloads and
        // then to "Completed" when it finishes.
        reloadStatusEntry.set("Requested");
        inst.flush();

        // Wait up to 15 minutes for DeepBlueSim to be ready, reminding the user every 10 seconds.
        // On GitHub's MacOS Continuous Integration servers, it can take over 8 minutes for Webots
        // to start.
        var startedWaitingTimeMs = System.currentTimeMillis();
        var isReady = false;
        while (!isReady
                && System.currentTimeMillis() - startedWaitingTimeMs < 900000) {
            try {
                long elapsedTime =
                        System.currentTimeMillis() - startedWaitingTimeMs;
                long remainingTime = 900000 - elapsedTime;
                if (remainingTime > 0) {
                    isReady = isReadyFuture.get(10000, TimeUnit.MILLISECONDS);
                } else
                    break;
            } catch (TimeoutException ex) {
                System.err.println(userReminder);
            } catch (InterruptedException | ExecutionException e) {
                throw new RuntimeException(
                        "Error while waiting for Webots to be ready", e);
            }
        }
        if (!isReady) {
            throw new TimeoutException("Webots not ready in time");
        }
        pauser.setCallback(() -> {
            double simTimeSec = simTimeSecSubscriber.get();
            double robotTimeSec = robotTime.get();
            double deltaSecs = simTimeSec - robotTimeSec;
            // If we still haven't caught up to the simulator, then wait longer.
            // This would typically happen when robot time hasn't yet started.
            if (deltaSecs > 0) {
                pauser.stop();
                pauser.startSingle(deltaSecs);
                return;
            }
            // We're caught up, so pause and tell the sim what our new time is so that it can
            // continue.
            SimHooks.pauseTiming();
            robotTimeSecPublisher.set(robotTimeSec);
            inst.flush();
        });


        inst.addListener(simTimeSecSubscriber.getTopic(),
                EnumSet.of(Kind.kValueAll, Kind.kImmediate), (event) -> {
                    double simTimeSec = event.valueData.value.getDouble();
                    double robotTimeSec = robotTime.get();
                    // Ignore the default initial value
                    if (simTimeSec == -1.0) {
                        return;
                    }

                    // If we're not behind the sim time, there is nothing to do.
                    double deltaSecs = simTimeSec - robotTimeSec;
                    if (deltaSecs < 0.0) {
                        return;
                    }

                    // We are behind the sim time, so run until we've caught up.
                    // We use a Notifier instead of SimHooks.stepTiming() because
                    // using SimHooks.stepTiming() causes accesses to sim data to block.
                    pauser.stop();
                    pauser.startSingle(deltaSecs);
                    SimHooks.resumeTiming();
                });

        robotTimeSecPublisher.set(robotTime.get());
        inst.flush();

        System.out.println("Webots has started. Robot is running.");

        return this;
    }

    public WebotsManager runAutonomous(Measure<Time> runTime) {
        System.out.println("Enabling in autonomous.");
        // Simulate starting autonomous
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        try (Notifier endNotifier = new Notifier(() -> {
            System.out.println("Calling robot.endCompetition()");
            robot.endCompetition();
        })) {
            // HAL must be initialized or SimDeviceSim.resetData() will crash and SmartDashboard
            // might not work.
            HAL.initialize(500, 0);
            SimDeviceSim.resetData();
            endNotifier.startSingle(runTime.in(Seconds));
            robot.startCompetition();
        } finally {
            SimDeviceSim.resetData();
            // HAL.shutdown();
        }
        return this;
    }

    public WebotsManager withNodePosition(String defPath,
            Consumer<Translation3d> acceptor) {
        var watcher = new Watcher(defPath);
        var pos = watcher.getPosition();
        acceptor.accept(pos);
        return this;
    }
    /**
     * Closes this instance, freeing any resources that in holds.
     */
    public void close() {
        reloadStatusEntry.close();
        robotTimeSecPublisher.close();
        simTimeSecSubscriber.close();
        inst.close();
        inst.stopServer();
        pauser.close();
    }
}
