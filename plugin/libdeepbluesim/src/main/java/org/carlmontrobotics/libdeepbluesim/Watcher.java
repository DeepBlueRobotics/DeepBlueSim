package org.carlmontrobotics.libdeepbluesim;

import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.EnumSet;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

/**
 * Provides kinematic information for a Webots node.
 */
public class Watcher implements AutoCloseable {
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
    private Translation3d position = null;
    private CompletableFuture<Void> rotationReady = new CompletableFuture<>();
    private CompletableFuture<Void> velocityReady = new CompletableFuture<>();
    private final NetworkTableInstance inst;

    /**
     * Constructs an instance that watches a particular Webots node.
     *
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     */
    Watcher(String defPath) {
        inst = NetworkTableInstance.getDefault();
        NetworkTable table =
                inst.getTable("/DeepBlueSim/WatchedNodes/" + defPath);
        // Ask that the robot's position, rotation, and velocity be updated for us every
        // simulation step
        table.addListener(EnumSet.of(Kind.kValueRemote, Kind.kImmediate),
                (t, key, value) -> {
                    switch (key) {
                        case "position":
                            double[] posAsArray =
                                    value.valueData.value.getDoubleArray();
                            position = new Translation3d(posAsArray[0],
                                    posAsArray[1], posAsArray[2]);
                            positionReady.complete(null);
                            break;

                        case "rotation":
                            rotationReady.complete(null);
                            break;

                        case "velocity":
                            velocityReady.complete(null);
                            break;
                    }
                });
        positionTopic = table.getDoubleArrayTopic("position");
        positionPublisher = positionTopic.publish();
        positionTopic.setCached(false);
        positionPublisher.set(new double[] {});
        rotationTopic = table.getDoubleArrayTopic("rotation");
        rotationPublisher = rotationTopic.publish();
        rotationTopic.setCached(false);
        rotationPublisher.set(new double[] {});
        velocityTopic = table.getDoubleArrayTopic("velocity");
        velocityPublisher = velocityTopic.publish();
        velocityTopic.setCached(false);
        velocityPublisher.set(new double[] {});
    }

    /**
     * Gets the current position of the node.
     * 
     * @return the node's position.
     */
    public Translation3d getPosition() {
        if (!inst.isConnected()) {
            LOG.log(Level.DEBUG,
                    "NetworkTables is not connected, so starting server");
            inst.startServer();
        }
        if (LOG.isLoggable(Level.DEBUG))
            LOG.log(Level.DEBUG, "Waiting for position of %s to be ready"
                .formatted(positionTopic.getName()));
        positionReady.join();
        if (LOG.isLoggable(Level.DEBUG))
            LOG.log(Level.DEBUG,
                "Position of %s is ready".formatted(positionTopic.getName()));
        return position;
    }

    @Override
    public void close() {
        positionPublisher.close();
        rotationPublisher.close();
        velocityPublisher.close();
    }
}
