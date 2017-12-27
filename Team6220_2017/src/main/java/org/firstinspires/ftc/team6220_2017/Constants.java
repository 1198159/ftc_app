package org.firstinspires.ftc.team6220_2017;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    // Standard conversions (for AndyMark 40's)
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1120;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;
    public static final float MM_PER_INCH = 25.4f;


    // Robot specifications
    public static final float WHEEL_DIAMETER_MM = 4 * MM_PER_INCH;    // 4 inch diameter wheel
    public static final double GEAR_RATIO = 48.0 / 84.0;            // Driven to driving gear
    // todo Factor in front is empirical; it isn't necessarily theoretically correct
    // Note:  larger factor = shorter distance, smaller factor = longer distance
    public static final double MM_PER_ANDYMARK_TICK = (Math.PI * WHEEL_DIAMETER_MM) / (ANDYMARK_TICKS_PER_ROTATION * GEAR_RATIO);


    // Drive control constants-----------------------
    public static final double MINIMUM_DRIVE_POWER = 0.07;
    public static final double MINIMUM_TURNING_POWER = 0.1;
    public static final double ANGLE_TOLERANCE_DEG = 2.5;
    public static final double POSITION_TOLERANCE_MM = 10.0;   // todo Adjust
    // Constants for adjusting powers that are proportional to angle and position differences
    public static final double TURNING_POWER_FACTOR = 1.0 / 70;    // todo Adjust
    public static final double DRIVE_POWER_FACTOR = 1.0 / 700;    // todo Adjust
    //------------------------------------------------


    // todo Adjust
    // PID loop constants-------------------------------
    public static final double ROTATION_P = TURNING_POWER_FACTOR;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.01;
    public static final double TRANSLATION_P = DRIVE_POWER_FACTOR;
    public static final double TRANSLATION_I = 0;    // 0.0005  todo Adjust
    public static final double TRANSLATION_D = 0;    // 0.008  todo Adjust
    //---------------------------------------------------


    // Field specs
    public static final float MM_FIELD_SIZE = (12 * 12) * MM_PER_INCH;


    // Vuforia constants
    public static final int JEWEL_SAMPLE_LENGTH = 64;
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;


    // todo Adjust
    // Times the robot takes to drive to respective centers of cryptoboxes in autonomous
    public static final double STRAIGHT_BOX_TIME_RED = 1.27;    // todo Adjust
    public static final double STRAIGHT_BOX_TIME_BLUE = 1.55;    // todo Adjust
    public static final double ANGLED_BOX_TIME_BLUE = 0.85;  // todo Adjust
    public static final double ANGLED_BOX_TIME_RED = 0.62;   // todo Adjust
    public static final double STRAIGHT_COLUMN_DIFF = 0.3;
    public static final double ANGLED_COLUMN_DIFF = 0.34;


    // Drive mode constants
    public static final double SLOW_MODE_T_FACTOR = 0.3;
    public static final double SLOW_MODE_R_FACTOR = 0.35;
    public static final double T_FACTOR = 0.6;
    public static final double R_FACTOR = 0.55;


    // Servo positions--------------------------------
    public static final double LATERAL_JEWEL_SERVO_NEUTRAL = 0.27;
    public static final double LATERAL_JEWEL_SERVO_LEFT = 0.15;
    public static final double LATERAL_JEWEL_SERVO_RIGHT = 0.39;
    public static final double VERTICAL_JEWEL_SERVO_RETRACTED = 0.35;
    public static final double VERTICAL_JEWEL_SERVO_DEPLOYED = 0.69;
    public static final double VERTICAL_JEWEL_SERVO_INIT = 0.3;

    // todo Adjust
    public static final double WRIST_SERVO_DEPLOYED = 0.9;
    public static final double WRIST_SERVO_RETRACTED = 0.4;
    public static final double JOINT_SERVO_DEPLOYED = 1.0;
    public static final double JOINT_SERVO_RETRACTED = 0.075;
    //-------------------------------------------------


    // Ensure that input isn't used when no commands are given
    public static final double MINIMUM_JOYSTICK_POWER = 0.08;
    public static final double MINIMUM_TRIGGER_VALUE = 0.1;


    // Controls how much power is given to the arm
    public static final double ARM_POWER_CONSTANT = 0.7;

    // todo Adjust
    // Glyph mechanism scoring heights.  Numbers signify the number of glyphs stacked on top of
    // each other in a column.  Units are in andymark encoder ticks
     // Note:  2, 3, and 4 are slightly higher than necessary to accommodate uncertainty in glyph size
    public static final int HEIGHT_1 = -6500;         // Subtract 1280 to calculated value
    public static final int HEIGHT_2 = -5377;         // Subtract 1200 to calculated value
    public static final int HEIGHT_3 = -4513;         // Subtract 1270 to calculated value
    public static final int HEIGHT_4 = -3484;         // Subtract 1330 to calculated value
}
