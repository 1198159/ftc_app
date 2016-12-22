package org.firstinspires.ftc.team6220;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1140;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;

    public static final int LAUNCHER_PULLBACK_POSITION = -720;
    public static final int LAUNCHER_FIRING_POSITION = -335;

    public static final int LAUNCHER_TRIM_INTERVAL = -50;
    public static final double LAUNCHER_TRIM_POWER = 0.1;
    public static final double LAUNCHER_SHOOT_POWER = 1.0;

    public static final double DEGREE_TO_RADIAN = Math.PI / 180;

    //used for PID loops involving rotation and turning power adjustments
    public static final double TURNING_POWER_FACTOR = 1.0 / 500;

    public static final double MINIMUM_TURNING_POWER = 0.07;
    public static final double ANGLE_TOLERANCE = 3.0;
    public static final double POSITION_TOLERANCE = .010;

    public static final double GATE_SERVO_RETRACTED_POSITION = 0.0;
    public static final double GATE_SERVO_DEPLOYED_POSITION = 0.48;
    public static final double GATE_SERVO_CLOSE_DELAY = 0.4;


}
