package org.firstinspires.ftc.team6220_2017;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    // Standard conversions
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1120;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;
    public static final float MM_PER_INCH = 25.4f;


    // Robot specifications
    public static final float WHEEL_DIAMETER_MM = 4 * MM_PER_INCH;    // 4 inch diameter wheel
    public static final double GEAR_RATIO = 48.0 / 84.0;            // Driven to driving gear
    //the numerator and the denominator might be switched, but it seems to be going the correct distance
    public static final double MM_PER_ANDYMARK_TICK = (ANDYMARK_TICKS_PER_ROTATION * GEAR_RATIO)/(Math.PI * WHEEL_DIAMETER_MM);


    // Drive control constants-----------------------
    public static final double MINIMUM_DRIVE_POWER = 0.05;
    public static final double MINIMUM_TURNING_POWER = 0.1;
    public static final double ANGLE_TOLERANCE_DEG = 2.5;
    public static final double POSITION_TOLERANCE_MM = 15.0;   // todo Adjust
    // Constants for adjusting powers that are proportional to angle and position differences
    public static final double TURNING_POWER_FACTOR = 1.0 / 100;    // todo Adjust
    public static final double DRIVE_POWER_FACTOR = 1.0 / 1800;    // todo Adjust
    //------------------------------------------------


    // todo Adjust these experimentally
    // PID loop constants-------------------------------
    public static final double ROTATION_P = 0.8;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.2;
    public static final double TRANSLATION_P = 0.55;
    public static final double TRANSLATION_I = 0.005;
    public static final double TRANSLATION_D = 0.2;
    //---------------------------------------------------


    // Field specs
    public static final float MM_FIELD_SIZE = (12 * 12) * MM_PER_INCH;


    // Vuforia constants
    public static final int JEWEL_SAMPLE_LENGTH = 64;
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;


    // Drive mode constants
    public static final double SLOW_MODE_T_FACTOR = 0.2;
    public static final double SLOW_MODE_R_FACTOR = 0.48;
    public static final double T_FACTOR = 0.8;
    public static final double R_FACTOR = 0.68;


    // Servo positions--------------------------------
    public static final double LATERAL_JEWEL_SERVO_NEUTRAL = 0.27;
    public static final double LATERAL_JEWEL_SERVO_LEFT = 0.18;
    public static final double LATERAL_JEWEL_SERVO_RIGHT = 0.36;
    public static final double VERTICAL_JEWEL_SERVO_RETRACTED = 0.3;
    public static final double VERTICAL_JEWEL_SERVO_DEPLOYED = 0.69;

    // todo Adjust
    public static final double WRIST_SERVO_DEPLOYED = 0.9;
    public static final double WRIST_SERVO_RETRACTED = 0.4;
    public static final double JOINT_SERVO_DEPLOYED = 1.0;
    public static final double JOINT_SERVO_RETRACTED = 0.075;
    //-------------------------------------------------


    // Ensure that input isn't used when no commands are given
    public static final double MINIMUM_JOYSTICK_POWER = 0.05;
    public static final double MINIMUM_TRIGGER_VALUE = 0.1;


    // Controls how much power is given to the arm
    public static final double ARM_POWER_CONSTANT = 0.7;

}
