package org.firstinspires.ftc.team6220_2017;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    //# of encoder ticks per revolution for different motor types
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1140;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;

    public static final double DEGREES_TO_RADIANS = Math.PI / 180;

    //robot tolerances used for autonomous
    public static final double MINIMUM_TURNING_POWER = 0.07;    //todo needs to be changed
    public static final double ANGLE_TOLERANCE = 2.5;   //todo needs to be changed
    public static final double POSITION_TOLERANCE = .015;   //todo needs to be changed

    //used for PID loops involving rotation and turning power adjustments
    public static final double TURNING_POWER_FACTOR = 1.0 / 700;    //not determined yet

    public static final float MM_FIELD_SIZE = 3657;
}
