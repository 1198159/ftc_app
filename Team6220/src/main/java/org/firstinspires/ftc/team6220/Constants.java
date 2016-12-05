package org.firstinspires.ftc.team6220;

/**
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    public static double degToRadConversionFactor = Math.PI / 180;

    //used for PID loops involving rotation and turning power adjustments
    public static double TURNING_POWER_FACTOR = 1 / 350;

    public static double MINIMUM_TURNING_POWER = 0.08;
    public static double MINIMUM_ANGLE_DIFF = 3.0;
    public static double X_TOLERANCE = .010;
    public static double Y_TOLERANCE = .010;
    public static double W_TOLERANCE = 3.0;

    public static double GATE_SERVO_RETRACTED_POSITION = 0.0;
    public static double GATE_SERVO_DEPLOYED_POSITION = 0.48;
    public static final double GATE_SERVO_CLOSE_DELAY = 0.4;
}
