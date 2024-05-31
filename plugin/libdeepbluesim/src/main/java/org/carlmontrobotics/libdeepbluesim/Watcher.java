package org.carlmontrobotics.libdeepbluesim;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Watcher {
    DoubleArrayTopic positionTopic, rotationTopic, velocityTopic;

    Watcher(String defPath) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("/WebotsSupervisor/" + defPath);
        // Ask that the robot's position, rotation, and velocity be updated for us every
        // simulation step
        positionTopic = table.getDoubleArrayTopic("position");
        positionTopic.publish().set(new double[] {});
        rotationTopic = table.getDoubleArrayTopic("rotation");
        rotationTopic.publish().set(new double[] {});
        velocityTopic = table.getDoubleArrayTopic("velocity");
        velocityTopic.publish().set(new double[] {});
    }

    public Translation3d getPosition() {
        double[] posAsArray = positionTopic.subscribe(null).get();
        return new Translation3d(posAsArray[0], posAsArray[1], posAsArray[2]);
    }
}
