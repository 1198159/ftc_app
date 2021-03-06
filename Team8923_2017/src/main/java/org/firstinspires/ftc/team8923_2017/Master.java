package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main robot code center, hold all the information needed to run the robot
 */

public abstract class Master extends LinearOpMode
{
    enum GGServoPositions
    {
        UPPERLEFTFULLOPEN(0.45),
        UPPERLEFTHALFOPEN(0.65),
        UPPERLEFTFULLCLOSED(0.77),

        UPPERRIGHTFULLOPEN(0.45),
        UPPERRIGHTHALFOPEN(0.35),
        UPPERRIGHTFULLCLOSED(0.30),

        LOWERLEFTFULLOPEN(0.45),
        LOWERLEFTHALFOPEN(0.35),
        LOWERLEFTFULLCLOSED(0.25),

        LOWERRIGHTFULLOPEN(0.50),
        LOWERRIGHTHALFOPEN(0.65),
        LOWERRIGHTFULLCLOSED(0.80);

        public final double val;
        GGServoPositions(double i) { val = i; }
        public final double val() { return val; }
    }

    // Declare motors here
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;
    DcMotor motorGG = null;
    DcMotor motorFF = null;

    // Declare servos here
    Servo servoJJ = null;
    Servo servoGGUL = null;
    Servo servoGGUR = null;
    Servo servoGGDL = null;
    Servo servoGGDR = null;

    // Declare any neccessary sensors here
    BNO055IMU imu;

    // Declare any robot-wide variables here
    double slowModeDivisor = 1.0;
    int GGZero = 0;
    int FFZero = 0;

    // Declare constants here
    private static  final double GEAR_RATIO = 1.0;
    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120.0; // Standard number for Andymark Neverest 40's
    private static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    private static final double WHEEL_DIAMETER = 4.0 * 25.4; // 4 inch diameter to MM = 101.6
    private static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;// = 319.024
    private static final double TURN_POWER_CONSTANT = 0.9;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;
    //Servos constants
    double SERVO_JJ_UP = 0.8; //Port 5, Hub 1
    double SERVO_JJ_DOWN = 0.15;
    double SERVO_JJ_MIDDLE = 0.5;
    double SERVO_JJ_MIDDLE1 = 0.45;
    double SERVO_JJ_MIDDLE2 = 0.4;
    double SERVO_JJ_MIDDLE3 = 0.35;
    double SERVO_JJ_MIDDLE4 = 0.3;
    double SERVO_JJ_MIDDLE5 = 0.27;

    int GGLiftTicks = 1700; // was 1700

    //declare IMU
    double currentRobotAngle;
    double jX;
    double jY;
    double kAngle;

    int newTargetFL;
    int newTargetFR;
    int newTargetBL;
    int newTargetBR;
    double currentFL;
    double currentFR;
    double currentBL;
    double currentBR;
    double moveErrorFL;
    double moveErrorFR;
    double moveErrorBL;
    double moveErrorBR;
    double speedFL;
    double speedFR;
    double speedBL;
    double speedBR;
    double kMove = 1/600.0;
    double TOL = 150.0;
    double AngleTOL = 3.0;
    double angleError;
    double pivot;
    double motorPowerFL;
    double motorPowerFR;
    double motorPowerBL;
    double motorPowerBR;


    public void InitHardware()
    {
        // Motors here
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorGG = hardwareMap.get(DcMotor.class, "motorGG");
        motorFF = hardwareMap.get(DcMotor.class, "motorFF");

        // Servos here
        servoJJ = hardwareMap.get(Servo.class, "servoJJ");
        servoGGUL = hardwareMap.get(Servo.class, "servoGGUL");
        servoGGUR = hardwareMap.get(Servo.class, "servoGGUR");
        servoGGDL = hardwareMap.get(Servo.class, "servoGGDL");
        servoGGDR = hardwareMap.get(Servo.class, "servoGGDR");

        motorGG.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorGG.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGG.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Sensors here
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        GGZero = motorGG.getCurrentPosition();
        motorGG.setTargetPosition(GGZero);

        FFZero = motorFF.getCurrentPosition();
        motorFF.getCurrentPosition();
    }

    // 45 denotes the angle at which the motors are mounted in referece to the chassis frame
    void driveOmni45(double driveAngle, double drivePower, double turnPower)
    {
        // Calculate out x and y directions of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        /*
         * to explain:
         * each wheel omni wheel exerts a force in one direction like tank but differs in the fact that they have rollers mounted perpendicular
         * to the wheels outer edge so they can passively roll at 90 degrees to the wheel's facing. This means that with the wheels mounted at 45
         * degrees to the chassis frame and assuming the left hand rule for the motors, each wheel's power needs to be 90 degrees out of phase
         * from the previous wheel on the unit circle starting with positive y and x for FL and going clockwise around the unit circle and robot
         * from there
         */

        double powerFL = y + x + (turnPower * TURN_POWER_CONSTANT);
        double powerFR = -y + x + (turnPower * TURN_POWER_CONSTANT);
        double powerBL = y - x + (turnPower * TURN_POWER_CONSTANT);
        double powerBR = -y - x + (turnPower * TURN_POWER_CONSTANT);

        double scalar = Math.max(Math.abs(powerFL),  Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        if(scalar < 1)
            scalar = 1;

        powerFL /= (scalar * slowModeDivisor);
        powerFR /= (scalar * slowModeDivisor);
        powerBL /= (scalar * slowModeDivisor);
        powerBR /= (scalar * slowModeDivisor);

        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }



    // normalizing the angle to be between -180 to 180
    public double adjustAngles(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }

    void stopDriving()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide || pad.left_trigger > 0.35
                || pad.right_trigger > 0.35);
    }

    boolean motorIsAtTarget(DcMotor motor)
    {
        // Return false if the motor isn't in RTP mode
        if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            return false;

        int tolerance = 10;
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < tolerance;
    }
}
