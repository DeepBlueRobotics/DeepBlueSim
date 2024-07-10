package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.carlmontrobotics.libdeepbluesim.WebotsSimulator;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.api.parallel.ResourceLock;

import java.lang.System.Logger;
import java.lang.System.Logger.Level;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;

@Timeout(value = 30, unit = TimeUnit.MINUTES)
@ResourceLock("WebotsSimulator")
public class PWMMotorControllerTest {
    // NOTE: By default, only messages at INFO level or higher are logged. To change that, if you
    // are using the default system logger, edit the logging properties file specified by the
    // java.util.logging.config.file system property so that both ".level=FINE" and
    // "java.util.logging.ConsoleHandler.level=FINE". For tests via Gradle, the
    // java.util.logging.config.file system property can be configured using the systemProperty of
    // the test task.
    private static final Logger LOG =
            System.getLogger(PWMMotorControllerTest.class.getName());

    // Should be the same as the basicTimeStep in the world's WorldInfo (default to 32ms)
    private double simStepSizeSecs = 0.032;

    // Derivation of the math used below:
    // Let tau be the torque applied by the motor (Nm).
    // Let J be the moment of inertia of the load (kg m^2).
    // Let alpha be the angular accleration of the load (rad/s^2).
    // Let w be the speed of the load (rad/s).
    //
    // From rotational mechanics:
    // tau = J*alpha
    // alpha = dw/dt
    // dw/dt = tau / J
    //
    // Let V be the applied voltage
    // Let V_max be the motor's rated voltage (e.g. 12V)
    // Let i_free be the motor's free current (A) which is needed to overcome friction
    // Let w_free be the motor's free speed
    // Let i_stall be the motor's stall current
    // Let tau_stall be the motor's stall torque
    // Let R = V_max/i_stall (ie. the motor's resistance)
    // Let k_T = tau_stall/i_stall (ie. the motor's torque constant) (Nm/A)
    // Let k_v = w_free/(V_max - R*i_free) (ie. the motor's velocity constant) ((rad/s)/Volt)
    //
    // From electromagnetism:
    // tau = i*k_T
    // i = (V-w/k_v)/R
    //
    // Substitution gives:
    // dw/dt = (V-w/k_v)/R*k_T/J
    //
    // Rearranging gives:
    // dw/dt = k_T/(R*J)*V - k_T/(R*J)/k_v*w
    // dw/dt + k_T/(R*J)/k_v*w = k_T/(R*J)*V

    // The following w(t) solves that differential equation (where w_0 = w(0)):
    // w(t) = w_0 + (V*k_v-w_0)(1-exp(-k_T/(R*J*k_v)*t))
    //
    // That is an exponential decay from w_0 to V*k_v with a time constant of:
    // t_c = R*J*k_v/k_T
    //
    // Let w_f = V*k_v (ie the asymptotic speed that corresponds to a voltage of V)
    //
    // That can be rearranged to give:
    // w(t) = w_f + (w_0 - w_f) * exp(-t/t_c)
    //
    // Integrating with respect to t gives:
    // theta(t) = C + w_f*t - (w_0 - w_f) * t_c * exp(-t/t_c)
    //
    // C is a constant of integration that we can determine from initial conditions:
    // theta(0) = theta_0 = C - (w_0 - w_f) * t_c
    // C = theta_0 + (w_0 - w_f)) * t_c
    //
    // Substituting gives:
    // theta(t) = theta_0 + (w_0 - w_f) * t_c + w_f*t - (w_0 - w_f) * t_c * exp(-t/t_c)
    //
    // Rearranging gives:
    // theta(t) = theta_0 + w_f*t + (w_0 - w_f) * t_c * (1-exp(-t/t_c))

    private double computeSpeedRadPerSec(DCMotor gearMotor, double moiKgM2,
            double throttle, double initialSpeedRadPerSec, double tSecs) {
        double targetSpeedRadPerSec = throttle * gearMotor.nominalVoltageVolts
                * gearMotor.KvRadPerSecPerVolt;
        double timeConstantSecs = computeTimeConstantSecs(gearMotor, moiKgM2);
        // w(t) = w_f + (w_0 - w_f) * exp(-t/t_c)
        return targetSpeedRadPerSec
                + (initialSpeedRadPerSec - targetSpeedRadPerSec)
                        * Math.exp(-tSecs / timeConstantSecs);
    }

    private double computeAngleRadians(DCMotor gearMotor, double moiKgM2,
            double throttle, double initialSpeedRadPerSec,
                    double initialAngleRad, double tSecs) {
        double timeConstantSecs = computeTimeConstantSecs(gearMotor, moiKgM2);
        double targetSpeedRadPerSec = throttle * gearMotor.nominalVoltageVolts
                * gearMotor.KvRadPerSecPerVolt;
        // theta(t) = theta_0 + w_f*t + (w_0 - w_f) * t_c * (1-exp(-t/t_c))
        return initialAngleRad + targetSpeedRadPerSec * tSecs
                + (initialSpeedRadPerSec - targetSpeedRadPerSec)
                        * timeConstantSecs
                        * (1 - Math.exp(-tSecs / timeConstantSecs));
    }

    private double computeTimeConstantSecs(DCMotor gearMotor, double moiKgM2) {
        // t_c = R*J*k_v/k_T
        return gearMotor.rOhms * moiKgM2 * gearMotor.KvRadPerSecPerVolt
                / gearMotor.KtNMPerAmp;
    }

    private double computeCylinderMoiKgM2(double radiusMeters,
            double heightMeters, double densityKgPerM3) {
        double massKg = densityKgPerM3 * Math.PI * radiusMeters * radiusMeters
                * heightMeters;
        return massKg * radiusMeters * radiusMeters / 2.0;
    }

    private double timeOfNextStepSecs(double tSecs) {
        return Math.ceil(tSecs / simStepSizeSecs) * simStepSizeSecs;
    }

    private double expectedSpeedRadPerSec(DCMotor gearMotor, double moiKgM2,
            double tSecs) {
        // The robot autonomous code runs the motor at 0.5 for 2 seconds and then stops it (i.e.
        // sets the speed to 0 and lets it stop on it's own).
        double tMotorStopsSecs = timeOfNextStepSecs(2.0);

        if (tSecs <= tMotorStopsSecs) {
            return computeSpeedRadPerSec(gearMotor, moiKgM2,
                    0.5, 0, tSecs);
        }
        return computeSpeedRadPerSec(gearMotor, moiKgM2, 0,
                expectedSpeedRadPerSec(gearMotor, moiKgM2, tMotorStopsSecs),
                tSecs - tMotorStopsSecs);
    }

    private double expectedAngleRadians(DCMotor gearMotor, double moiKgM2,
            double tSecs) {
        // The robot autonomous code runs the motor at 0.5 for 2 seconds and then stops it (i.e.
        // sets the speed to 0 and lets it stop on it's own).
        double tMotorStopsSecs = timeOfNextStepSecs(2.0);
        if (tSecs <= tMotorStopsSecs) {
            return computeAngleRadians(gearMotor, moiKgM2,
                    0.5, 0, 0, tSecs);
        }
        return computeAngleRadians(gearMotor, moiKgM2, 0,
                expectedSpeedRadPerSec(gearMotor, moiKgM2, tMotorStopsSecs),
                expectedAngleRadians(gearMotor, moiKgM2, tMotorStopsSecs),
                tSecs - tMotorStopsSecs);
    }

    private void assertShaftCorrectAtSecs(DCMotor gearMotor, double moiKgM2,
            double actualSpeedRadPerSec, double actualAngleRadians,
            double tSecs) {
        // TODO: Make sim accurate enough that this can be less than 1*simStepSizeSecs
        // (https://github.com/DeepBlueRobotics/DeepBlueSim/issues/101)
        double jitterSecs = 5 * simStepSizeSecs;
        double expectedEarlierSpeedRadPerSec =
                expectedSpeedRadPerSec(gearMotor, moiKgM2, tSecs - jitterSecs);
        double expectedLaterSpeedRadPerSec =
                expectedSpeedRadPerSec(gearMotor, moiKgM2, tSecs + jitterSecs);
        LOG.log(Level.DEBUG, "expectedEarlierSpeedRadPerSec = {0}",
                expectedEarlierSpeedRadPerSec);
        LOG.log(Level.DEBUG, "expectedLaterSpeedRadPerSec = {0}",
                expectedLaterSpeedRadPerSec);
        assertEquals(
                (expectedEarlierSpeedRadPerSec + expectedLaterSpeedRadPerSec)
                        / 2,
                actualSpeedRadPerSec,
                Math.abs(expectedEarlierSpeedRadPerSec
                        - expectedLaterSpeedRadPerSec) / 2,
                "Shaft not close enough to target angular velocity");
        double expectedEarlierAngleRadians =
                expectedAngleRadians(gearMotor, moiKgM2, tSecs - jitterSecs);
        double expectedLaterAngleRadians =
                expectedAngleRadians(gearMotor, moiKgM2, tSecs + jitterSecs);
        double expectedAngleRadians =
                (expectedEarlierAngleRadians + expectedLaterAngleRadians) / 2;
        double toleranceRadians = Math.abs(
                expectedEarlierAngleRadians - expectedLaterAngleRadians) / 2;
        assertTrue(MathUtil.isNear(
                expectedAngleRadians,
                        actualAngleRadians,
                toleranceRadians, 0, 2 * Math.PI),
                "Shaft not close enough to target rotation. Expected %g +/- %g radians, but got %g radians."
                        .formatted(expectedAngleRadians, toleranceRadians,
                                actualAngleRadians));
    }

    private double computeGearing(DCMotor motor, double desiredFreeSpeedRPS) {
        return motor.freeSpeedRadPerSec / (2 * Math.PI) / desiredFreeSpeedRPS;
    }

    private double computeFlywheelThickness(DCMotor gearMotor,
            double flywheelRadiusMeters, double flywheelDensityKgPerM3,
            double desiredTimeConstantSecs) {
        // for a time constant of t_c seconds:
        // t_c = R*J*k_v/k_T
        // J = t_c*k_T/(k_v*R)
        // J = 0.5*density * pi * r^2 * h * r^2
        // h = 2*J/(density*pi*r^4)
        // h = 2*t_c*k_T/(k_v*R*density*pi*r^4)
        return 2 * desiredTimeConstantSecs * gearMotor.KtNMPerAmp
                / (gearMotor.KvRadPerSecPerVolt * gearMotor.rOhms
                        * flywheelDensityKgPerM3
                        * Math.PI * Math.pow(flywheelRadiusMeters, 4));
    }

    private static record Measurement(double simTimeSec,
            double speedRadPerSec) {
    }

    private Measurement m1, m2;

    @Test
    void testShaftRotatesInAutonomous() throws Exception {
        // To ensure we the flywheel doesn't spin or accelerate too fast...
        var desiredFlywheelFreeSpeedRPS = 1.0;
        var desiredTimeConstantSecs = 1.0;

        // For this test to pass, the motor and flywheel need to be setup in the world as follows.
        var flywheelRadiusMeters = 0.5;
        var flywheelDensityKgPerM3 = 1000.0;
        var motorModelName = "MiniCIM";
        var motor = (DCMotor) (DCMotor.class
                .getDeclaredMethod("get" + motorModelName, int.class)
                .invoke(null, 1));
        var gearing = computeGearing(motor, desiredFlywheelFreeSpeedRPS);
        var gearMotor = motor.withReduction(gearing);
        LOG.log(Level.INFO,
                "For a free speed of {0} RPS when using a {1}, assuming the gearing is {2}.\n",
                desiredFlywheelFreeSpeedRPS, motorModelName, gearing);
        var flywheelThicknessMeters = computeFlywheelThickness(
                gearMotor, flywheelRadiusMeters,
                flywheelDensityKgPerM3, desiredTimeConstantSecs);
        LOG.log(Level.INFO,
                "For a time constant of {0} second when using a {1} with a gearing of {2} and a flywheel with radius {3} meters and density {4} kg/m^3, assuming the flywheel thickness is {5} meters.\n",
                desiredTimeConstantSecs, motorModelName, gearing,
                flywheelRadiusMeters, flywheelDensityKgPerM3,
                flywheelThicknessMeters);

        // With those settings, the flywheel MOI will be:
        var moiKgM2 = computeCylinderMoiKgM2(flywheelRadiusMeters,
                flywheelThicknessMeters, flywheelDensityKgPerM3);

        try (var manager = new WebotsSimulator(
                "../plugin/controller/src/webotsFolder/dist/worlds/PWMMotorController.wbt",
                PWMMotorControllerRobot::new)) {
            manager.atSec(0.0, s -> {
                s.enableAutonomous();
            }).everyStep(s -> {
                LOG.log(Level.DEBUG,
                        "robotTime = {0}, simTimeSec = {1}, speedRadPerSec = {2}",
                        s.getRobotTimeSec(), s.getSimTimeSec(),
                        s.angularVelocity("SHAFT").getAngle());
            }).atSec(5 * simStepSizeSecs, s -> {
                m1 = new Measurement(s.getSimTimeSec(),
                        s.angularVelocity("SHAFT").getAngle());
            }).atSec(1.0, s -> {
                m2 = new Measurement(s.getSimTimeSec(),
                        s.angularVelocity("SHAFT").getAngle());
                assertCorrectTimeConstant(gearMotor, moiKgM2, 0.5);
                assertShaftCorrectAtSecs(gearMotor, moiKgM2,
                        s.angularVelocity("SHAFT").getAngle(),
                        s.rotation("SHAFT").getZ(), s.getRobotTimeSec());
            }).atSec(2.0 + 5 * simStepSizeSecs, s -> {
                m1 = new Measurement(s.getSimTimeSec(),
                        s.angularVelocity("SHAFT").getAngle());
            }).atSec(3.0, s -> {
                m2 = new Measurement(s.getSimTimeSec(),
                        s.angularVelocity("SHAFT").getAngle());
                assertCorrectTimeConstant(gearMotor, moiKgM2, 0.0);
                assertShaftCorrectAtSecs(gearMotor, moiKgM2,
                        s.angularVelocity("SHAFT").getAngle(),
                        s.rotation("SHAFT").getZ(), s.getRobotTimeSec());
            }).run();
        }
    }

    private void assertCorrectTimeConstant(DCMotor gearMotor, double moiKgM2,
            double throttle) {
        var targetSpeedRadPerSec = throttle * gearMotor.nominalVoltageVolts
                * gearMotor.KvRadPerSecPerVolt;
        // Use the 2 measurements to measure the time constant.
        // w(t) = w_f + (w_0 - w_f) * exp(-t/t_c)
        // Let w'(t) = w(t) - w_f = (w_0 - w_f) * exp(-t/t_c)
        // w'(t1) = (w_0 - w_f) * exp(-t1/t_c)
        // w'(t2) = (w_0 - w_f) * exp(-t2/t_c)
        // w'(t2)/w'(t1) = exp((t1-t2)/t_c)
        // ln(w'(t2)/w'(t1)) = (t1-t2)/t_c
        // t_c = (t1-t2)/ln(w'(t2)/w'(t1))
        var actualTimeConstantSecs = (m1.simTimeSec() - m2.simTimeSec())
                / (Math.log((m2.speedRadPerSec() - targetSpeedRadPerSec)
                        / (m1.speedRadPerSec() - targetSpeedRadPerSec)));
        assertEquals(computeTimeConstantSecs(gearMotor, moiKgM2),
                actualTimeConstantSecs, 0.02, "Time constant incorrect");
    }
}
