package org.team199.deepbluesim.mediators;

import java.util.Arrays;
import java.util.Comparator;

import org.team199.deepbluesim.Constants;
import org.team199.deepbluesim.Simulation;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.Field;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightMediator implements Runnable {

    private final Camera camera;
    private final Field selectedPipeline;
    private final NetworkTable ntTable;
    private final double cameraFOVRad;
    private final int cameraWidthPx, cameraHeightPx, defaultPipeline,
            numPipelines;

    private long lastPipeline = -1;

    // Camera Controls
    private final NetworkTableEntry pipeline;
    // Not Implemented: ledMode, stream, crop, camerapose_robotspace_set, robot_orientation_set,
    // fiducial_id_filters_set

    // Targeting Response Data
    private final NetworkTableEntry hasTarget, targetX, targetY, targetXNC,
            targetYNC, targetArea, targetShortSide, targetLongSide,
            targetHorizontalSide, targetVerticalSide, actualPipeline, heartbeat,
            targetCorners, rawTargets;
    // Not Implemented: tl, cl, json, tclass, tc, hw, apriltag/3d-data, rawfiducials

    public LimelightMediator(Camera camera, String ntTableName,
            double cameraFOVRad, int cameraWidthPx, int cameraHeightPx,
            int defaultPipeline, int numPipelines) {
        this.camera = camera;
        camera.recognitionEnable(Constants.sensorTimestep);

        selectedPipeline = Simulation.getSupervisor().getFromDevice(camera)
                .getField("selectedPipeline");

        this.cameraFOVRad = cameraFOVRad;
        this.cameraWidthPx = cameraWidthPx;
        this.cameraHeightPx = cameraHeightPx;
        this.defaultPipeline =
                defaultPipeline < 0 || defaultPipeline > (numPipelines - 1) ? 0
                        : defaultPipeline;
        this.numPipelines = numPipelines;

        ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        pipeline = ntTable.getEntry("pipeline");
        hasTarget = ntTable.getEntry("tv");
        targetX = ntTable.getEntry("tx");
        targetY = ntTable.getEntry("ty");
        targetXNC = ntTable.getEntry("txnc");
        targetYNC = ntTable.getEntry("tync");
        targetArea = ntTable.getEntry("ta");
        targetShortSide = ntTable.getEntry("tshort");
        targetLongSide = ntTable.getEntry("tlong");
        targetHorizontalSide = ntTable.getEntry("thor");
        targetVerticalSide = ntTable.getEntry("tvert");
        actualPipeline = ntTable.getEntry("getpipe");
        heartbeat = ntTable.getEntry("hb");
        targetCorners = ntTable.getEntry("tcornxy");
        rawTargets = ntTable.getEntry("rawtargets");

        if (numPipelines == 0) {
            // No available pipeline. We can't do anything :/
            setNoTargets();
            actualPipeline.setInteger(0);
        } else {
            Simulation.registerPeriodicMethod(this);
        }
    }

    @Override
    public void run() {
        long requestedPipeline = pipeline.getInteger(defaultPipeline);
        if (requestedPipeline != lastPipeline) {
            lastPipeline = requestedPipeline;
            if (requestedPipeline < 0
                    || requestedPipeline > (numPipelines - 1)) {
                requestedPipeline = defaultPipeline;
            }
            selectedPipeline.setSFInt32(Math.toIntExact(requestedPipeline));
            actualPipeline.setInteger(requestedPipeline);
        }

        // Update Targeting Results
        CameraRecognitionObject[] objects = camera.getRecognitionObjects();
        // Limelight always chooses the largest object
        // It also has some code to try to focus on one object if two have a similar size
        // (to prevent flipping), but I think that that is unnecessary in this simple implementation
        Arrays.sort(objects, Comparator.comparingInt(
                obj -> obj.getSizeOnImage()[0] * obj.getSizeOnImage()[1]));
    }

    private void setNoTargets() {
        hasTarget.setBoolean(false);
        // TODO: OTHER STUFF
    }

}
