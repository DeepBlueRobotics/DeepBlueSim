package org.carlmontrobotics.deepbluesim;

import java.util.Arrays;
import java.util.Optional;
import java.util.concurrent.CopyOnWriteArraySet;
import java.util.stream.IntStream;

import org.carlmontrobotics.deepbluesim.mediators.AnalogInputEncoderMediator;
import org.carlmontrobotics.deepbluesim.mediators.CANEncoderMediator;
import org.carlmontrobotics.deepbluesim.mediators.CANMotorMediator;
import org.carlmontrobotics.deepbluesim.mediators.DutyCycleMediator;
import org.carlmontrobotics.deepbluesim.mediators.GyroMediator;
import org.carlmontrobotics.deepbluesim.mediators.LimelightMediator;
import org.carlmontrobotics.deepbluesim.mediators.PWMMotorMediator;
import org.carlmontrobotics.deepbluesim.mediators.PlayingWithFusionTimeOfFlightMediator;
import org.carlmontrobotics.deepbluesim.mediators.WPILibEncoderMediator;
import org.carlmontrobotics.wpiws.devices.AnalogInputSim;
import org.carlmontrobotics.wpiws.devices.CANAnalogInputSim;
import org.carlmontrobotics.wpiws.devices.CANEncoderSim;
import org.carlmontrobotics.wpiws.devices.CANMotorSim;
import org.carlmontrobotics.wpiws.devices.DutyCycleSim;
import org.carlmontrobotics.wpiws.devices.EncoderSim;
import org.carlmontrobotics.wpiws.devices.PWMSim;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.Device;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Field;
import com.cyberbotics.webots.controller.Gyro;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Node;
import com.cyberbotics.webots.controller.PositionSensor;
import com.cyberbotics.webots.controller.Supervisor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class SimRegisterer {

    private static final CopyOnWriteArraySet<PositionSensor> unboundEncoders =
            new CopyOnWriteArraySet<>();

    public static void connectDevices() {
        Supervisor robot = Simulation.getSupervisor();

        boolean hasGyro = false;

        for (int i = 0; i < robot.getNumberOfDevices(); i++) {
            Device device = robot.getDeviceByIndex(i);
            String name = device.getName();
            if (name.startsWith("DBSim_")) {
                try {
                    String type = name.split("_")[1];
                    switch (type) {
                        case "Encoder":
                            connectEncoder((PositionSensor) device);
                            break;
                        case "Limelight":
                            connectLimelight((Camera) device);
                            break;
                        case "Motor":
                            connectMotor((Motor) device);
                            break;
                        case "PlayingWithFusionTimeOfFlight":
                            connectPlayingWithFusionTimeOfFlight(
                                    (DistanceSensor) device);
                            break;
                    }
                } catch (Exception e) {
                    System.err.println("Error occurred connecting to device " + device.getName() + ":");
                    e.printStackTrace(System.err);
                    System.err.flush();
                }
            }

            if (device instanceof Gyro) {
                if (hasGyro) {
                    System.err.println("Warning: multiple gyros detected! Only one will be used.");
                } else {
                    new GyroMediator((Gyro) device);
                    hasGyro = true;
                }
            }

            if(!unboundEncoders.isEmpty()) {
                Simulation.registerPeriodicMethod(SimRegisterer::tryBindEncoders);
            }
        }
    }

    public static void tryBindEncoders() {
        if(unboundEncoders.isEmpty()) return;

        PositionSensor[] unboundEncodersCopy =
                unboundEncoders.toArray(new PositionSensor[0]);
        unboundEncoders.clear();

        for (PositionSensor device : unboundEncodersCopy) {
            try {
                connectEncoder(device);
            } catch (Exception e) {
                System.err.println("Error occurred connecting to device "
                        + device.getName() + ":");
                e.printStackTrace(System.err);
                System.err.flush();
            }
        }
    }

    public static void connectEncoder(PositionSensor device) {
        Node node = Simulation.getSupervisor().getFromDevice(device);

        String[] nameParts = device.getName().split("_");
        boolean isOnMotorShaft = Boolean.parseBoolean(nameParts[2]);
        boolean isAbsolute = Boolean.parseBoolean(nameParts[3]);
        double absoluteOffsetDeg = Double.parseDouble(nameParts[4]);
        boolean isInverted = Boolean.parseBoolean(nameParts[5]);
        int countsPerRevolution = Integer.parseInt(nameParts[6]);

        double gearing;
        try {
            String motorName = device.getMotor().getName();
            if(motorName.startsWith("DBSim_Motor")) {
                gearing = Double.parseDouble(motorName.split("_")[4]);
            } else {
                throw new IllegalArgumentException();
            }
        } catch(Exception e) {
            System.err.println("Warning: No valid motor found for encoder \"" + device.getName() + "\"! Assuming 1:1 gearing...");
            gearing = 1;
        }

        if (node.getField("channelA") != null) { // WPILib Encoder
            int channelA = node.getField("channelA").getSFInt32();
            int channelB = node.getField("channelB").getSFInt32();

            // WPILib encoders no longer have deterministic names (based on channel numbers), so we have to search for the encoder
            // This was the best way I could think of to do it
            Optional<EncoderSim> simDevice = Arrays.stream(EncoderSim.enumerateDevices()).map(EncoderSim::new)
                .filter(encoder -> encoder.getChannelA() == channelA && encoder.getChannelB() == channelB)
                .findAny();

            if(simDevice.isPresent()) {
                new WPILibEncoderMediator(device, simDevice.get(), isOnMotorShaft, isInverted, countsPerRevolution, gearing);
            } else {
                unboundEncoders.add(device);
            }
        } else if(node.getField("id") != null) { // CANCoder
            new DutyCycleMediator(device, new DutyCycleSim("CANDutyCycle:CANCoder[" + node.getField("id").getSFInt32() + "]", "SimDevice"), isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
        } else if(node.getTypeName().startsWith("SparkMax")) { // One of the SparkMax encoder types
            Motor motor = device.getMotor();

            String motorName;
            if(motor == null || !(motorName = motor.getName()).startsWith("DBSim_Motor_Spark")) {
                System.err.println("Warning: Spark Max Encoder \"" + device.getName() + "\" is not attached to a Spark Max motor!");
                return;
            }

            String[] motorNameParts = motorName.split("_");
            int motorId = Integer.parseInt(motorNameParts[3]);

            if (node.getTypeName().equals("SparkMaxAlternateEncoder")) {
                String simDeviceName = "CANEncoder:CANSparkMax[" + motorId + "]-alternate";
                new CANEncoderMediator(device, new CANEncoderSim(simDeviceName, "SimDevice"), isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
            } else if (node.getTypeName().equals("SparkMaxAbsoluteEncoder")) {
                String simDeviceName = "CANDutyCycle:CANSparkMax[" + motorId + "]";
                new DutyCycleMediator(device, new DutyCycleSim(simDeviceName, "SimDevice"), isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
            } else if (node.getTypeName().equals("SparkMaxAnalogSensor")) {
                String simDeviceName = "CANAIn:CANSparkMax[" + motorId + "]";
                new AnalogInputEncoderMediator(device, new AnalogInputSim(simDeviceName, "SimDevice"), isOnMotorShaft, isAbsolute, absoluteOffsetDeg, isInverted, countsPerRevolution, gearing);
            }
        } else {
            System.err.println("Warning: Ignoring invalid encoder: " + device.getName() + "!");
        }
    }

    public static void connectLimelight(Camera device) {
        Node node = Simulation.getSupervisor().getFromDevice(device);

        String[] nameParts = device.getName().split("_", 8);
        double cameraFOVRad = Double.parseDouble(nameParts[2]);
        int cameraWidthPx = Integer.parseInt(nameParts[3]);
        int cameraHeightPx = Integer.parseInt(nameParts[4]);
        int defaultPipeline = Integer.parseInt(nameParts[5]);
        String name = nameParts[6];

        Field pipelinesField = node.getField("pipelines");
        int numPipelines = pipelinesField.getCount();
        double[][] pipelines = IntStream.range(0, numPipelines)
                .mapToObj(pipelinesField::getMFColor).toArray(double[][]::new);

        // The "-sim" is added to prevent conflicts with actual limelight tables
        // (See https://github.com/DeepBlueRobotics/lib199/issues/102#issuecomment-2150499942)
        new LimelightMediator(device, name + "-sim", cameraFOVRad,
                cameraWidthPx, cameraHeightPx, pipelines, defaultPipeline);
    }

    public static void connectMotor(Motor device) {
        String[] nameParts = device.getName().split("_");
        String controllerType = nameParts[2];
        int port = Integer.parseInt(nameParts[3]);
        double gearing = Double.parseDouble(nameParts[4]);
        boolean inverted = Boolean.parseBoolean(nameParts[5]);
        double nominalVoltageVolts = Double.parseDouble(nameParts[6]);
        double stallTorqueNewtonMeters = Double.parseDouble(nameParts[7]);
        double stallCurrentAmps = Double.parseDouble(nameParts[8]);
        double freeCurrentAmps = Double.parseDouble(nameParts[9]);
        double freeSpeedRPM = Double.parseDouble(nameParts[10]);

        DCMotor motorConstants = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, Units.rotationsPerMinuteToRadiansPerSecond(freeSpeedRPM), 1);

        if(controllerType.equals("PWM")) {
            new PWMMotorMediator(device, new PWMSim(Integer.toString(port)), motorConstants, gearing, inverted);
        } else {
            String simDeviceName = "CAN" + controllerType.replaceAll("\\s", "")
                    + "[" + port + "]";
            new CANMotorMediator(device, new CANMotorSim(simDeviceName),
                    motorConstants, gearing, inverted);
        }
    }

    public static void connectPlayingWithFusionTimeOfFlight(
            DistanceSensor device) {
        String port = device.getName().split("_")[2];

        String baseDeviceName =
                "PlayingWithFusionTimeOfFlight[%s]".formatted(port);
        CANAnalogInputSim rangeDevice =
                new CANAnalogInputSim(baseDeviceName + "-rangeVoltsIsMM");
        CANAnalogInputSim ambientLightLevelDevice = new CANAnalogInputSim(
                baseDeviceName + "-ambientLightLevelVoltsIsMcps");
        new PlayingWithFusionTimeOfFlightMediator(device, rangeDevice,
                ambientLightLevelDevice);
    }

}
