package org.firstinspires.ftc.team6220_2017;

/*
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    // # of encoder ticks per revolution for different motor types
    public static final int  ANDYMARK_TICKS_PER_ROTATION = 1140;
    public static final int TETRIX_TICKS_PER_ROTATION = 1440;

    public static final double DEGREES_TO_RADIANS = Math.PI / 180;

    // Robot tolerances used for autonomous
    public static final double MINIMUM_TURNING_POWER = 0.07;    //todo needs to be changed
    public static final double ANGLE_TOLERANCE = 2.0;   //todo needs to be changed
    public static final double POSITION_TOLERANCE = .015;   //todo needs to be changed

    // Used for PID loops involving rotation and turning power adjustments
    public static final double TURNING_POWER_FACTOR = 1.0 / 700;    //not determined yet

    public static final float MM_PER_INCH = 25.4f;

    // Field specs
    public static final float MM_FIELD_SIZE = (12 * 12) * MM_PER_INCH;

    // Vuforia constants
    public static final int JEWEL_SAMPLE_LENGTH = 64;
    public static final int IMAGE_WIDTH = 1280;
    public static final int IMAGE_HEIGHT = 720;

    // Drive mode constants
    public static final double SLOW_MODE_T_FACTOR = 0.2;
    public static final double SLOW_MODE_R_FACTOR = 0.3;
    public static final double T_FACTOR = 0.85;
    public static final double R_FACTOR = 0.65;

    //todo adjust
    // Servo positions
    public static final double LATERAL_JEWEL_SERVO_NEUTRAL = 0.25;
    public static final double LATERAL_JEWEL_SERVO_LEFT = 0.18;
    public static final double LATERAL_JEWEL_SERVO_RIGHT = 0.32;
    public static final double VERTICAL_JEWEL_SERVO_RETRACTED = 0.35;
    public static final double VERTICAL_JEWEL_SERVO_DEPLOYED = 0.66;


    public static final double WRIST_SERVO_DEPLOYED = 0.9;
    public static final double WRIST_SERVO_RETRACTED = 0.4;
    public static final double JOINT_SERVO_DEPLOYED = 1.0;
    public static final double JOINT_SERVO_RETRACTED = 0.075;
    //

    // Ensure that input isn't used when no commands are given
    public static final double MINIMUM_JOYSTICK_POWER = 0.05;
    public static final double MINIMUM_TRIGGER_VALUE = 0.1;

    // Controls how much power is given to the arm
    public static final double ARM_POWER_CONSTANT = 0.7;

    //public static final double CLAW_INCREMENT = 0.1;

}
