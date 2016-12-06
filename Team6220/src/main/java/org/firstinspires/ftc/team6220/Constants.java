package org.firstinspires.ftc.team6220;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    public static final double DEGREE_TO_RADIAN = Math.PI / 180;

    //used for PID loops involving rotation and turning power adjustments
    public static final double TURNING_POWER_FACTOR = 1 / 350;

    public static final double MINIMUM_TURNING_POWER = 0.08;
    public static final double ANGLE_TOLERANCE = 3.0;
    public static final double POSITION_TOLERANCE = .010;

    public static final double GATE_SERVO_RETRACTED_POSITION = 0.0;
    public static final double GATE_SERVO_DEPLOYED_POSITION = 0.48;
    public static final double GATE_SERVO_CLOSE_DELAY = 0.4;
}
