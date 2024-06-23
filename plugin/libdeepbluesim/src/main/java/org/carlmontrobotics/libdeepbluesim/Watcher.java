package org.carlmontrobotics.libdeepbluesim;

import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

/**
 * Provides kinematic information for a Webots node.
 */
class Watcher {
    // NOTE: By default, only messages at INFO level or higher are logged. To change that, if you
    // are using the default system logger, edit the logging properties file specified by the
    // java.util.logging.config.file system property so that both ".level=FINE" and
    // "java.util.logging.ConsoleHandler.level=FINE". For tests via Gradle, the
    // java.util.logging.config.file system property can be configured using the systemProperty of
    // the test task.
    private static final Logger LOG = System.getLogger(Watcher.class.getName());

    private DoubleArrayTopic positionTopic, rotationTopic, velocityTopic;
    private DoubleArrayPublisher positionPublisher, rotationPublisher,
            velocityPublisher;
    private CompletableFuture<Void> positionReady = new CompletableFuture<>();
    private volatile Translation3d position = null;
    private volatile Rotation3d rotation = null;
    private CompletableFuture<Void> rotationReady = new CompletableFuture<>();
    private CompletableFuture<Void> velocityReady = new CompletableFuture<>();
    private final NetworkTableInstance inst;
    private final String defPath;
    private final NetworkTable table;

    private static Map<String, Watcher> watcherByDefPath = new HashMap<>();

    static void closeAll() {
        for (var watcher : watcherByDefPath.values()) {
            watcher.positionPublisher.close();
            watcher.rotationPublisher.close();
            watcher.velocityPublisher.close();
        }
        watcherByDefPath.clear();
    }


    /**
     * Gets a watcher that watches a particular Webots node.
     *
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     */
    static Watcher getByDefPath(String defPath) {
        return watcherByDefPath.computeIfAbsent(defPath, (dp) -> {
            return new Watcher(dp);
        });
    }

    /**
     * Constructs an instance that watches a particular Webots node.
     *
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     */
    private Watcher(String defPath) {
        this.defPath = defPath;
        inst = NetworkTableInstance.getDefault();
        table = inst.getTable("/DeepBlueSim/WatchedNodes/" + defPath);
        // Ask that the robot's position, rotation, and velocity be updated for us every
        // simulation step
        table.addListener(EnumSet.of(Kind.kValueRemote), (t, key, value) -> {
            synchronized (Watcher.class) {
                switch (key) {
                    case "position":
                        double[] posAsArray =
                                value.valueData.value.getDoubleArray();
                        position = new Translation3d(posAsArray[0],
                                posAsArray[1], posAsArray[2]);
                        positionReady.complete(null);
                        break;

                    case "rotation":
                        double[] rotAsArray =
                                value.valueData.value.getDoubleArray();
                        rotation = new Rotation3d(rotAsArray[0], rotAsArray[1],
                                rotAsArray[2]);
                        rotationReady.complete(null);
                        break;

                    case "velocity":
                        velocityReady.complete(null);
                        break;
                }
            }
                });
        var pubSubOptions = new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                PubSubOption.keepDuplicates(true), // including duplicates
                PubSubOption.periodic(Double.MIN_VALUE), // ASAP
        };
        synchronized (Watcher.class) {
            positionTopic = table.getDoubleArrayTopic("position");
            positionPublisher = positionTopic.publish(pubSubOptions);
            positionTopic.setCached(false);
            positionPublisher.set(dummyPosition);
            rotationTopic = table.getDoubleArrayTopic("rotation");
            rotationPublisher = rotationTopic.publish(pubSubOptions);
            rotationTopic.setCached(false);
            rotationPublisher.set(dummyRotation);
            velocityTopic = table.getDoubleArrayTopic("velocity");
            velocityPublisher = velocityTopic.publish(pubSubOptions);
            velocityTopic.setCached(false);
            velocityPublisher.set(dummyVelocity);
            inst.flush();
        }
    }

    static private final double[] dummyPosition = new double[] {0, 0, 0},
            dummyRotation = new double[] {0, 0, 0},
            dummyVelocity = new double[] {0, 0, 0, 0, 0, 0};

    /**
     * Gets the current position of the node.
     * 
     * @return the node's position.
     */
    Translation3d getPosition() {
        if (!inst.isConnected()) {
            LOG.log(Level.DEBUG,
                    "NetworkTables is not connected, so starting server");
            inst.startServer();
        }
        if (LOG.isLoggable(Level.DEBUG))
            LOG.log(Level.DEBUG, "Waiting for position of %s to be ready"
                    .formatted(defPath));
        positionReady.join();
        if (LOG.isLoggable(Level.DEBUG))
            LOG.log(Level.DEBUG, "Position of %s is ready".formatted(defPath));
        return position;
    }

    /**
     * Gets the current rotation of the node.
     * 
     * @return the node's rotation.
     */
    Rotation3d getRotation() {
        if (!inst.isConnected()) {
            LOG.log(Level.DEBUG,
                    "NetworkTables is not connected, so starting server");
            inst.startServer();
        }
        if (LOG.isLoggable(Level.DEBUG))
            LOG.log(Level.DEBUG, "Waiting for rotation of %s to be ready"
                    .formatted(defPath));
        rotationReady.join();
        if (LOG.isLoggable(Level.DEBUG))
            LOG.log(Level.DEBUG, "Rotation of %s is ready".formatted(defPath));
        return rotation;
    }
}
