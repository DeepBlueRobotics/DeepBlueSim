package org.carlmontrobotics.deepbluesim.mediators;

import java.util.Arrays;
import java.util.Comparator;

import org.carlmontrobotics.deepbluesim.Constants;
import org.carlmontrobotics.deepbluesim.Simulation;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public class LimelightMediator implements Runnable {

    /**
     * The "distance" between the actual and expected R, G, and B values within which a target will
     * be considered to be of the desired color (in %). Note: For a target to be considered, it must
     * have a recognition color whose R, G, and B are all (independently) within this range of the
     * expected R, G, and B.
     */
    public static final double ACCURACY_THRESHOLD = 3.0 / 255.0;

    private final Camera camera;
    private final double[][] availablePipelines;
    private final NetworkTable ntTable;
    private final int cameraWidthPx, cameraHeightPx, defaultPipeline;
    private final double viewPlaneHalfWidth, viewPlaneHalfHeight, cameraAreaPx2;

    // Camera Controls
    // Note: streamEnabled is not part of the standard limelight API. It can be set to "true" to
    // enable the camera overlay in Webots. Make sure that Overlays > Hide All Camera Overlays is
    // disabled and (Right click on robot in scene tree) > Overlays > Camera Devices >
    // <limelight name> is enabled
    private final BooleanSubscriber streamEnabled;
    private final DoubleSubscriber pipeline;
    // Not Implemented: ledMode, stream, crop, camerapose_robotspace_set, robot_orientation_set,
    // fiducial_id_filters_set

    // Targeting Response Data
    private final DoublePublisher hasTarget, targetX, targetY, targetXNC,
            targetYNC, targetArea, targetShortSide, targetLongSide,
            targetHorizontalSide, targetVerticalSide, actualPipeline, heartbeat;
    private final StringPublisher targetClass;
    private final DoubleArrayPublisher targetColor, targetCorners, rawTargets;
    // Not Implemented: tl, cl, json, hw, apriltag/3d-data, rawfiducials

    private boolean streamCurrentlyEnabled;
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
        streamEnabled =
                ntTable.getBooleanTopic("streamEnabled").subscribe(false);
        pipeline =
                ntTable.getDoubleTopic("pipeline").subscribe(defaultPipeline);
        hasTarget = ntTable.getDoubleTopic("tv").publish();
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
        // Update Values from Controls
        if (streamEnabled.get() != streamCurrentlyEnabled) {
            streamCurrentlyEnabled = !streamCurrentlyEnabled;
            if (streamCurrentlyEnabled) {
                camera.enable(Constants.sensorTimestep);
            } else {
                camera.disable();
            }
        }

        double requestedPipeline = pipeline.get();
        if (requestedPipeline != lastPipeline) {
            lastPipeline = requestedPipeline;
            // The pipeline should always be an int, but Limelight docs say to use a double.
            // If it's not an int, or it's out of range, fallback on the default
            if ((int) requestedPipeline != requestedPipeline
                    || requestedPipeline < 0
                    || requestedPipeline >= availablePipelines.length) {
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
                    for (int i = 0; i < objectColors.length; i += 3) {
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
                        // size (to prevent flipping), but I think that that is unnecessary in this
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
        double tx = Math.toDegrees(Math
                .atan2(normalizedObjectPosition[0] * viewPlaneHalfWidth, 1));
        double ty = Math.toDegrees(Math
                .atan2(normalizedObjectPosition[1] * viewPlaneHalfHeight, 1));
        double ta = (objectSize[0] * objectSize[1]) / cameraAreaPx2;

        // Old code used for determining tc as an average of the object recognition colors.
        // For the moment, I've decided to just send the actual color that was detected
        // (which is approximated by actualPipeline) because, imo, this makes more sense. If it's
        // desireable to restore the averaging functionality, this code can be uncommented (and the
        // inner comment on numberOfColors removed?) and targetColor (below) can be set to
        // objectColors

        // // Determine Average Target Color
        // double[] objectColors = primaryObject.getColors();
        // // See https://github.com/cyberbotics/webots/pull/6564
        // // int numberOfColors = primaryObject.getNumberOfColors();
        // int numberOfColors = 1;

        // double[] averageColor = new double[3];
        // for (int i = 0; i < objectColors.length; i += 3) {
        // averageColor[0] += objectColors[i + 0]; // r
        // averageColor[1] += objectColors[i + 1]; // g
        // averageColor[2] += objectColors[i + 2]; // b
        // }
        // averageColor[0] /= numberOfColors;
        // averageColor[1] /= numberOfColors;
        // averageColor[2] /= numberOfColors;

        // Publish results
        hasTarget.set(1);
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
        targetColor.set(activePipeline);
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
        double[] rawTargetData = Arrays.stream(objects).map(object -> {
            int[] targetPosition = object.getPositionOnImage();
            int[] targetSize = object.getSizeOnImage();
            double[] normalizedTargetPosition =
                    getNormalizedPositionOfObject(targetPosition, targetSize);
            double targetArea = (targetSize[0] * targetSize[1]) / cameraAreaPx2;
            return new double[] {normalizedTargetPosition[0],
                    normalizedTargetPosition[1], targetArea};
        }).flatMapToDouble(Arrays::stream).toArray();

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
        hasTarget.set(0);
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
        targetColor.set(new double[0]);
        targetCorners.set(new double[0]);
        rawTargets.set(new double[0]);
    }

    private void updateHeartbeat() {
        heartbeatValue++;
        heartbeatValue %= 2e9;
        heartbeat.set(heartbeatValue);
    }

}
