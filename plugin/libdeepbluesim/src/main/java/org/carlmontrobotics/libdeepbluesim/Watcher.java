package org.carlmontrobotics.libdeepbluesim;

import java.util.EnumSet;
import java.util.concurrent.CompletableFuture;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;

/**
 * Provides kinematic information for a Webots node.
 */
public class Watcher {
    private DoubleArrayTopic positionTopic, rotationTopic, velocityTopic;
    private CompletableFuture<Void> positionReady = new CompletableFuture<>();
    private CompletableFuture<Void> rotationReady = new CompletableFuture<>();
    private CompletableFuture<Void> velocityReady = new CompletableFuture<>();

    /**
     * Constructs an instance that watches a particular Webots node.
     *
     * @param defPath the DEF path (i.e. a dot separated path of DEF names, like "ROBOT.ARM.ROLLER")
     *        of the node to watch.
     */
    Watcher(String defPath) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table =
                inst.getTable("/DeepBlueSim/WatchedNodes/" + defPath);
        // Ask that the robot's position, rotation, and velocity be updated for us every
        // simulation step
        table.addListener(EnumSet.of(Kind.kValueRemote), (t, key, value) -> {
            switch (key) {
                case "position":
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
        positionTopic.publish().set(new double[] {});
        rotationTopic = table.getDoubleArrayTopic("rotation");
        rotationTopic.publish().set(new double[] {});
        velocityTopic = table.getDoubleArrayTopic("velocity");
        velocityTopic.publish().set(new double[] {});
    }

    /**
     * Gets the current position of the node.
     * 
     * @return the node's position.
     */
    public Translation3d getPosition() {
        positionReady.join();
        double[] posAsArray = positionTopic.subscribe(null).get();
        return new Translation3d(posAsArray[0], posAsArray[1], posAsArray[2]);
    }
}
