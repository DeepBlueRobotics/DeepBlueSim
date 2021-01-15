/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package code;

public final class Constants {
    public static final double motorGearing = 6.8;
    // Max speed of a NEO in rad/s. Used for specifying motor velocity in Webots
    public static final double neoMotorConstant = (5676 / motorGearing) * (Math.PI * 2) / 60.;
    public static final double wheelDiameter = 5 * 0.0254;
    public static final double maxSpeed = neoMotorConstant * wheelDiameter / 2;
    public static final double wheelBase = 0.46101;
    public static final double trackWidth = 0.45085;

    public static final int joystickPort = 0;

    public static class CANPorts {
        public static final int dtFrontLeft = 0;
        public static final int dtFrontRight = 1;
        public static final int dtBackLeft = 2;
        public static final int dtBackRight = 3;
    }
}
