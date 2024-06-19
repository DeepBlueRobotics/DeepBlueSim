package org.team199.deepbluesim.mediators;

import java.awt.Color;

import java.util.Arrays;
import java.util.Comparator;

import org.team199.deepbluesim.Constants;
import org.team199.deepbluesim.Simulation;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LimelightMediator implements Runnable {

    public static final double ACCURACY_THRESHOLD = 3.0 / 255.0;

    private final Camera camera;
    private final double[][] availablePipelines;
    private final NetworkTable ntTable;
    private final int cameraWidthPx, cameraHeightPx, defaultPipeline;
    private final double viewPlaneHalfWidth, viewPlaneHalfHeight, cameraAreaPx2;

    // Camera Controls
    private final DoubleSubscriber pipeline;
    // Not Implemented: ledMode, stream, crop, camerapose_robotspace_set, robot_orientation_set,
    // fiducial_id_filters_set

    // Targeting Response Data
    private final BooleanPublisher hasTarget;
    private final DoublePublisher targetX, targetY, targetXNC, targetYNC,
            targetArea, targetShortSide, targetLongSide, targetHorizontalSide,
            targetVerticalSide, actualPipeline, heartbeat;
    private final StringPublisher targetClass;
    private final DoubleArrayPublisher targetColor, targetCorners, rawTargets;
    // Not Implemented: tl, cl, json, hw, apriltag/3d-data, rawfiducials

    private double lastPipeline = -1;
    private double heartbeatValue = 0;
    private double[] activePipeline = new double[3];

    public LimelightMediator(Camera camera, String ntTableName,
            double cameraFOVRad, int cameraWidthPx, int cameraHeightPx,
            double[][] availablePipelines, int defaultPipeline) {
        this.camera = camera;
        camera.recognitionEnable(Constants.sensorTimestep);

        this.cameraWidthPx = cameraWidthPx;
        this.cameraHeightPx = cameraHeightPx;
        this.availablePipelines = availablePipelines;
        this.defaultPipeline = MathUtil.clamp(defaultPipeline, 0,
                availablePipelines.length - 1);

        // This is the code you get if you just follow the math in the docs, but we can optimize it
        // by shifting the order of operations
        //
        // cameraHorizontalFOVRad = cameraFOVRad;
        // cameraVerticalFOVRad = 2.0 * Math
        // .atan((Math.tan(cameraHorizontalFOVRad * 0.5) * cameraHeightPx)
        // / cameraWidthPx);
        //
        // viewPlaneHalfWidth = Math.tan(cameraHorizontalFOVRad * 0.5);
        // viewPlaneHalfHeight = Math.tan(cameraVerticalFOVRad * 0.5);

        viewPlaneHalfWidth = Math.tan(cameraFOVRad * 0.5);
        viewPlaneHalfHeight =
                (viewPlaneHalfWidth * cameraHeightPx) / cameraWidthPx;

        cameraAreaPx2 = cameraWidthPx * cameraHeightPx;

        ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        pipeline =
                ntTable.getDoubleTopic("pipeline").subscribe(defaultPipeline);
        hasTarget = ntTable.getBooleanTopic("tv").publish();
        targetX = ntTable.getDoubleTopic("tx").publish();
        targetY = ntTable.getDoubleTopic("ty").publish();
        targetXNC = ntTable.getDoubleTopic("txnc").publish();
        targetYNC = ntTable.getDoubleTopic("tync").publish();
        targetArea = ntTable.getDoubleTopic("ta").publish();
        targetShortSide = ntTable.getDoubleTopic("tshort").publish();
        targetLongSide = ntTable.getDoubleTopic("tlong").publish();
        targetHorizontalSide = ntTable.getDoubleTopic("thor").publish();
        targetVerticalSide = ntTable.getDoubleTopic("tvert").publish();
        actualPipeline = ntTable.getDoubleTopic("getpipe").publish();
        targetClass = ntTable.getStringTopic("tclass").publish();
        heartbeat = ntTable.getDoubleTopic("hb").publish();
        targetColor = ntTable.getDoubleArrayTopic("tc").publish();
        targetCorners = ntTable.getDoubleArrayTopic("tcornxy").publish();
        rawTargets = ntTable.getDoubleArrayTopic("rawtargets").publish();

        if (availablePipelines.length == 0) {
            // There are no available pipelines, so we can't do anything :(
            setNoTargets();
            Simulation.registerPeriodicMethod(this::updateHeartbeat);
        } else {
            Simulation.registerPeriodicMethod(this);
        }
    }

    @Override
    public void run() {
        double requestedPipeline = pipeline.get();
        if (requestedPipeline != lastPipeline) {
            lastPipeline = requestedPipeline;
            if ((int) requestedPipeline != requestedPipeline
                    || requestedPipeline < 0
                    || requestedPipeline >= availablePipelines.length - 1) {
                // The pipeline should always be an int, but Limelight docs say to use a double.
                // If it's not an int, or it's out of range, fallback on the default
                requestedPipeline = defaultPipeline;
            }
            activePipeline = availablePipelines[(int) requestedPipeline];
            actualPipeline.set(requestedPipeline);
        }



        // Update Primary Target Results

        // Get targets
        CameraRecognitionObject[] objects =
                Arrays.stream(camera.getRecognitionObjects()).filter(object -> {
                    double[] objectColors = object.getColors();
                    for (int i = 0; i < objectColors.length; i++) {
                        if (Math.abs(activePipeline[0]
                                - objectColors[i + 0]) < ACCURACY_THRESHOLD
                                && Math.abs(activePipeline[1] - objectColors[i
                                        + 1]) < ACCURACY_THRESHOLD
                                && Math.abs(activePipeline[2] - objectColors[i
                                        + 2]) < ACCURACY_THRESHOLD)
                            return true;
                    }
                    return false;
                })
                        // Limelight always chooses the largest object
                        // It also has some code to try to focus on one object if two have a similar
                        // size
                        // (to prevent flipping), but I think that that is unnecessary in this
                        // simple implementation
                        .sorted(Comparator
                                .comparing(
                                        CameraRecognitionObject::getSizeOnImage,
                                        Comparator.comparingInt(
                                                size -> size[0] * size[1]))
                                .reversed())
                        .toArray(CameraRecognitionObject[]::new);
        if (objects.length == 0) {
            setNoTargets();
            updateHeartbeat();
            return;
        }

        CameraRecognitionObject primaryObject = objects[0];

        // Determine Target Position
        int[] objectPosition = primaryObject.getPositionOnImage();
        int[] objectSize = primaryObject.getSizeOnImage();
        double[] normalizedObjectPosition =
                getNormalizedPositionOfObject(objectPosition, objectSize);

        // Modified from Limelight Docs: From Pixels to Angles
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory#from-pixels-to-angles
        double tx =
                Math.atan2(1, normalizedObjectPosition[0] * viewPlaneHalfWidth);
        double ty = Math.atan2(1,
                normalizedObjectPosition[1] * viewPlaneHalfHeight);
        double ta = (objectSize[0] * objectSize[1]) / cameraAreaPx2;

        // Determine Target Colors
        double[] objectColors = primaryObject.getColors();
        int numberOfColors = primaryObject.getNumberOfColors();

        double[] averageColor = new double[3];
        for (int i = 0; i < objectColors.length; i += 3) {
            averageColor[0] += objectColors[i + 0]; // r
            averageColor[1] += objectColors[i + 1]; // g
            averageColor[2] += objectColors[i + 2]; // b
        }
        averageColor[0] /= numberOfColors;
        averageColor[1] /= numberOfColors;
        averageColor[2] /= numberOfColors;
        // HSV is the same as HSB
        float[] averageColorHSV =
                Color.RGBtoHSB((int) Math.round(averageColor[0]),
                        (int) Math.round(averageColor[1]),
                        (int) Math.round(averageColor[2]), null);

        // Publish results
        hasTarget.set(true);
        targetX.set(tx);
        targetY.set(ty);
        // While real cameras have a primary pixel that's not the center, this is a simulation, so
        // we can say they're the same
        targetXNC.set(tx);
        targetYNC.set(ty);
        targetArea.set(ta);
        targetShortSide.set(Math.min(objectSize[0], objectSize[1]));
        targetLongSide.set(Math.max(objectSize[0], objectSize[1]));
        targetHorizontalSide.set(objectSize[0]);
        targetVerticalSide.set(objectSize[1]);
        targetClass.set(primaryObject.getModel());
        targetColor.set(new double[] {averageColorHSV[0], averageColorHSV[1],
                averageColorHSV[2]});
        targetCorners.set(new double[] {
                // Top Left
                objectPosition[0], objectPosition[1],
                // Top Right
                objectPosition[0] + objectSize[0], objectPosition[1],
                // Bottom Left
                objectPosition[0], objectPosition[1] + objectSize[1],
                // Bottom Right
                objectPosition[0] + objectPosition[1],
                objectPosition[1] + objectSize[1]});



        // Update Raw Target Results
        double[] rawTargetData = new double[9];

        // The primary object always exists
        rawTargetData[0] = normalizedObjectPosition[0];
        rawTargetData[1] = normalizedObjectPosition[1];
        rawTargetData[2] = ta;

        // If there's a second largest object, send data for that too
        if (objects.length > 1) {
            CameraRecognitionObject object2 = objects[1];
            int[] object2Position = object2.getPositionOnImage();
            int[] object2Size = object2.getSizeOnImage();
            double[] normalizedObject2Position =
                    getNormalizedPositionOfObject(object2Position, object2Size);
            rawTargetData[3] = normalizedObject2Position[0];
            rawTargetData[4] = normalizedObject2Position[1];
            rawTargetData[5] =
                    (object2Size[0] * object2Size[1]) / cameraAreaPx2;
        }

        // Same for the third largest
        if (objects.length > 1) {
            CameraRecognitionObject object3 = objects[2];
            int[] object3Position = object3.getPositionOnImage();
            int[] object3Size = object3.getSizeOnImage();
            double[] normalizedObject3Position =
                    getNormalizedPositionOfObject(object3Position, object3Size);
            rawTargetData[6] = normalizedObject3Position[0];
            rawTargetData[7] = normalizedObject3Position[1];
            rawTargetData[8] =
                    (object3Size[0] * object3Size[1]) / cameraAreaPx2;
        }

        rawTargets.set(rawTargetData);



        updateHeartbeat();
    }

    private double[] getNormalizedPositionOfObject(int[] objectPosition,
            int[] objectSize) {

        // Modified from Limelight Docs: From Pixels to Angles
        // https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory#from-pixels-to-angles

        double px = objectPosition[0] + (objectSize[0] / 2.0);
        double py = objectPosition[1] + (objectSize[1] / 2.0);
        double nx = (2.0 / cameraWidthPx) * (px - ((cameraWidthPx - 1) / 2.0));
        double ny =
                (2.0 / cameraHeightPx) * (((cameraHeightPx - 1) / 2.0) - py);
        return new double[] {nx, ny};
    }

    private void setNoTargets() {
        hasTarget.set(false);
        targetX.set(0);
        targetY.set(0);
        targetXNC.set(0);
        targetYNC.set(0);
        targetArea.set(0);
        targetShortSide.set(0);
        targetLongSide.set(0);
        targetHorizontalSide.set(0);
        targetVerticalSide.set(0);
        targetClass.set("");
        targetColor.set(new double[3]);
        targetCorners.set(new double[8]);
        rawTargets.set(new double[9]);
    }

    private void updateHeartbeat() {
        heartbeatValue++;
        heartbeatValue %= 2e9;
        heartbeat.set(heartbeatValue);
    }

}
