package org.firstinspires.ftc.team6220_2017;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    // Standard conversions
     // This is for an Andymark 40; 20's and 60's are different
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1120;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;
    public static final float MM_PER_INCH = 25.4f;


    // Robot specifications
    public static final float WHEEL_DIAMETER_MM = 4 * MM_PER_INCH;    // 4 inch diameter wheel
    public static final double GEAR_RATIO = 48.0 / 84.0;            // Driven to driving gear
    // This value is about 0.499 mm per tick; experimental results yield an actual value of 0.483
    public static final double MM_PER_ANDYMARK_TICK = (Math.PI * WHEEL_DIAMETER_MM) / (ANDYMARK_TICKS_PER_ROTATION * GEAR_RATIO);


    // Drive mode constants
    public static final double SLOW_MODE_T_FACTOR = 0.35;
    public static final double SLOW_MODE_R_FACTOR = 0.25;
    public static final double T_FACTOR = 0.7;
    public static final double R_FACTOR = 0.72;


    // Tolerances
    public static final double ANGLE_TOLERANCE_DEG = 2.5;
    public static final double POSITION_TOLERANCE_MM = 15.0;
    public static final double GLYPHTER_TOLERANCE_TICKS = 15;    // todo Adjust


    // Movement control constants-----------------------
    public static final double MINIMUM_DRIVE_POWER = 0.15;   // todo Adjust
    public static final double MINIMUM_TURNING_POWER = 0.1;
    public static final double MINIMUM_GLYPHTER_POWER = 0.1;   // todo Adjust
     // Constants for adjusting powers that are proportional to angle and position differences
    public static final double TURNING_POWER_FACTOR = 0.014;
    public static final double DRIVE_POWER_FACTOR = 0.0016;
    //------------------------------------------------


    // todo Adjust
    // PID loop constants-------------------------------
    public static final double ROTATION_P = TURNING_POWER_FACTOR;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.016;
    public static final double TRANSLATION_P = DRIVE_POWER_FACTOR;
    public static final double TRANSLATION_I = 0.00000002;    // todo Adjust
    public static final double TRANSLATION_D = 0.004;
    public static final double GLYPHTER_P = 0.0012;    // todo Adjust
    public static final double GLYPHTER_I = 0.0000001;    // todo Adjust
    public static final double GLYPHTER_D = 0;    // todo Adjust
    //---------------------------------------------------


    // Acceleration loop constants-------------------------------
    public static final double NAV_ACCEL = 0.0007;    // todo Adjust
    public static final double NAV_DECEL = 0.01;    // todo Adjust
    public static final double DRIVE_ACCEL = 0.002;    // todo Adjust
    public static final double DRIVE_DECEL = 0.01;    // todo Adjust
    public static final double TURN_ACCEL = 0.0025;    // todo Adjust
    public static final double TURN_DECEL = 0.01;    // todo Adjust
    //---------------------------------------------------


    // Field specs
    public static final float MM_FIELD_SIZE = (12 * 12) * MM_PER_INCH;


    // Vuforia constants
    public static final int JEWEL_SAMPLE_LENGTH = 64;
    public static final int JEWEL_SAMPLE_HEIGHT = 32;
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;

    // todo Phase out for encoder values
    // Times the robot takes to drive to respective centers of cryptoboxes in autonomous
    public static final double STRAIGHT_BOX_TIME_RED = 1.27;    // todo Adjust
    public static final double STRAIGHT_BOX_TIME_BLUE = 1.55;    // todo Adjust
    public static final double ANGLED_BOX_TIME_BLUE = 0.85;  // todo Adjust
    public static final double ANGLED_BOX_TIME_RED = 0.62;   // todo Adjust
    public static final double STRAIGHT_COLUMN_TIME_DIFF = 0.3;
    public static final double ANGLED_COLUMN_TIME_DIFF = 0.34;


    // Distances in encoder ticks the robot must drive to respective centers of cryptoboxes in autonomous
    public static final double STRAIGHT_BOX_DISTANCE_RED = 1007;
    public static final double STRAIGHT_BOX_DISTANCE_BLUE = 930;
    public static final double ANGLED_BOX_DISTANCE_BLUE = 300;  // todo Adjust
    public static final double ANGLED_BOX_DISTANCE_RED = 450;   // todo Adjust
    public static final double STRAIGHT_COLUMN_DIFF = 180;
    public static final double ANGLED_COLUMN_DIFF = 180;   // todo Adjust


    // Servo positions--------------------------------
    public static final double LATERAL_JEWEL_SERVO_INIT = 0;
    public static final double LATERAL_JEWEL_SERVO_NEUTRAL = 0.26;
    public static final double LATERAL_JEWEL_SERVO_LEFT = 0.17;
    public static final double LATERAL_JEWEL_SERVO_RIGHT = 0.42;
    public static final double VERTICAL_JEWEL_SERVO_RETRACTED = 0.52;
    public static final double VERTICAL_JEWEL_SERVO_DEPLOYED = 0.20;
    public static final double VERTICAL_JEWEL_SERVO_INIT = 0.49;

    public static final double GLYPH_CLIP_SERVO_RETRACTED = 0;   // todo Adjust
    public static final double GLYPH_CLIP_SERVO_DEPLOYED = 0.38;   // todo Adjust

    public static final double WRIST_SERVO_INIT = 1.0;   // todo Adjust
    public static final double WRIST_SERVO_RETRACTED = 0.89;   // todo Adjust
    public static final double WRIST_SERVO_DEPLOYED = 0.5;   // todo Adjust
    public static final double WRIST_SERVO_INCREMENT = 0.005;   // todo Adjust
    public static final double GRABBER_SERVO_GRIP = 0.43;   // todo Adjust
    public static final double GRABBER_SERVO_RELEASE = 0.22;   // todo Adjust
    public static final double GRABBER_SERVO_INCREMENT = 0.05;   // todo Adjust
    //-------------------------------------------------


    // Controls how much power is given to the arm
    public static final double ARM_POWER_CONSTANT = 1.0;


    // Ensure that input isn't used when no commands are given
    public static final double MINIMUM_JOYSTICK_POWER = 0.05;
    public static final double MINIMUM_TRIGGER_VALUE = 0.33;    // todo How large does this value need to be to prevent twitching?


    // todo Adjust
    // Glyph mechanism scoring heights.  Numbers signify the number of glyphs stacked on top of
    // each other in a column.  Units are in andymark encoder ticks
     // Note:  2, 3, and 4 are slightly higher than necessary to accommodate uncertainty in glyph size
    public static final int HEIGHT_1 = -11630/*-6500*/;         // Calculated value:  11037
    public static final int HEIGHT_2 = -9280/*-5377*/;         // Calculated value:  9273
    public static final int HEIGHT_3 = -7200/*-4513*/;         // Calculated value:  8004
    public static final int HEIGHT_4 = -6100/*-3484*/;         // Calculated value:  6519


    // Glyph rotation mechanism constants
     // Tells the glyph rotation mechanism how long it should rotate to accomplish a full turn
    public static final double TURNTABLE_ROTATION_TIME = 1.2;  // todo Adjust
    public static final double MINIMUM_TURNTABLE_POWER = 0.05;  // todo Adjust
}
