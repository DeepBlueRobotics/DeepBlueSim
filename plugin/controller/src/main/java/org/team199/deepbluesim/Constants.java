/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.team199.deepbluesim;

public final class Constants {
    public static enum Preset {
        SWERVE;
    }

    public static final Preset preset = Preset.SWERVE;

    public static final double motorGearing = 6.8;
    // Max speed of a NEO in rad/s. Used for specifying motor velocity in Webots
    public static final double neoMotorConstant = (5676 / motorGearing) * (Math.PI * 2) / 60.;
    public static final double wheelDiameter = 5 * 0.0254;
    public static final double maxSpeed = neoMotorConstant * wheelDiameter / 2;
    public static final double wheelBase = 0.46101;
    public static final double trackWidth = 0.45085;

    // Ports for the turn motors on the swerve drivetrain
    public static final int turnPorts[] = {4, 11, 5, 10};
    // The PPR for the quadrature encoder on a PG71 gearbox is 7. There are four pulses per edge = 28 EPR
    public static final int pg71EPR = 28;
    // The maximum analog encoder voltages for the swerve drivetrain MA3 absolute encoders
    public static final double maxEncVoltages[] = {2.858, 2.845, 2.829, 2.839};
}
