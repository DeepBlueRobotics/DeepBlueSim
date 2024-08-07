package org.carlmontrobotics.libdeepbluesim;

import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.CompletableFuture;

import org.carlmontrobotics.libdeepbluesim.internal.NTConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
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
    private volatile CompletableFuture<Void> positionReady =
            new CompletableFuture<>();
    private volatile CompletableFuture<Void> rotationReady =
            new CompletableFuture<>();
    private volatile CompletableFuture<Void> velocityReady =
            new CompletableFuture<>();
    private volatile Translation3d position = null, velocity = null;
    private volatile Vector<N3> angularVelocity = null;
    private volatile Rotation3d rotation = null;
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
        var watcher = watcherByDefPath.computeIfAbsent(defPath, Watcher::new);
        watcher.reset();
        return watcher;
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
        table = inst
                .getTable(NTConstants.WATCHED_NODES_TABLE_NAME + "/" + defPath);
        // Ask that the robot's position, rotation, and velocity be updated for us every
        // simulation step
        table.addListener(EnumSet.of(Kind.kValueRemote), (t, key, value) -> {
            synchronized (this) {
                switch (key) {
                    case NTConstants.POSITION_TOPIC_NAME:
                        double[] posAsArray =
                                value.valueData.value.getDoubleArray();
                        position = new Translation3d(posAsArray[0],
                                posAsArray[1], posAsArray[2]);
                        positionReady.complete(null);
                        break;

                    case NTConstants.ROTATION_TOPIC_NAME:
                        double[] rotAsArray =
                                value.valueData.value.getDoubleArray();
                        rotation = new Rotation3d(rotAsArray[0], rotAsArray[1],
                                rotAsArray[2]);
                        rotationReady.complete(null);
                        break;

                    case NTConstants.VELOCITY_TOPIC_NAME:
                        double[] velAsArray =
                                value.valueData.value.getDoubleArray();
                        velocity = new Translation3d(velAsArray[0],
                                velAsArray[1], velAsArray[2]);
                        angularVelocity = VecBuilder.fill(velAsArray[3],
                                        velAsArray[4], velAsArray[5]);
                        velocityReady.complete(null);
                        break;
                }
            }
        });
        var pubSubOptions = new PubSubOption[] {PubSubOption.sendAll(true), // Send every update
                PubSubOption.keepDuplicates(true), // including duplicates
                PubSubOption.periodic(Double.MIN_VALUE), // ASAP
        };
        positionTopic =
                table.getDoubleArrayTopic(NTConstants.POSITION_TOPIC_NAME);
        positionPublisher = positionTopic.publish(pubSubOptions);
        positionTopic.setCached(false);
        rotationTopic =
                table.getDoubleArrayTopic(NTConstants.ROTATION_TOPIC_NAME);
        rotationPublisher = rotationTopic.publish(pubSubOptions);
        rotationTopic.setCached(false);
        velocityTopic =
                table.getDoubleArrayTopic(NTConstants.VELOCITY_TOPIC_NAME);
        velocityPublisher = velocityTopic.publish(pubSubOptions);
        velocityTopic.setCached(false);
    }

    private synchronized void reset() {
        positionReady = new CompletableFuture<>();
        rotationReady = new CompletableFuture<>();
        velocityReady = new CompletableFuture<>();
        positionPublisher.set(dummyPosition);
        rotationPublisher.set(dummyRotation);
        velocityPublisher.set(dummyVelocity);
        inst.flush();
    }

    private static final double[] dummyPosition = new double[] {0, 0, 0},
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
        LOG.log(Level.DEBUG, "Waiting for position of {0} to be ready",
                defPath);
        positionReady.join();
        LOG.log(Level.DEBUG, "Position of {0} is ready", defPath);
        return position;
    }

    /**
     * Gets the current velocity of the node.
     * 
     * @return the node's velocity.
     */
    Translation3d getVelocity() {
        if (!inst.isConnected()) {
            LOG.log(Level.DEBUG,
                    "NetworkTables is not connected, so starting server");
            inst.startServer();
        }
        LOG.log(Level.DEBUG, "Waiting for velocity of {0} to be ready",
                defPath);
        velocityReady.join();
        LOG.log(Level.DEBUG, "Velocity of {0} is ready", defPath);
        return velocity;
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
        LOG.log(Level.DEBUG, "Waiting for rotation of {0} to be ready",
                defPath);
        rotationReady.join();
        LOG.log(Level.DEBUG, "Rotation of {0} is ready", defPath);
        return rotation;
    }

    /**
     * Gets the current angular velocity of the node.
     * 
     * @return the node's angular velocity.
     */
    Vector<N3> getAngularVelocity() {
        if (!inst.isConnected()) {
            LOG.log(Level.DEBUG,
                    "NetworkTables is not connected, so starting server");
            inst.startServer();
        }
        LOG.log(Level.DEBUG, "Waiting for angular velocity of {0} to be ready",
                defPath);
        velocityReady.join();
        LOG.log(Level.DEBUG, "Angular velocity of {0} is ready", defPath);
        return angularVelocity;
    }
}
