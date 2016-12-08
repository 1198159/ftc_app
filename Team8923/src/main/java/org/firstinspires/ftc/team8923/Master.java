package org.firstinspires.ftc.team8923;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * This class contains all objects and methods that should be accessible by all OpModes
 */
abstract class Master extends LinearOpMode
{
    // Declares all hardware on robot
    DcMotor motorFL = null;
    DcMotor motorFR = null;
    DcMotor motorBL = null;
    DcMotor motorBR = null;
    DcMotor motorLift = null;
    DcMotor motorCollector = null;
    DcMotor motorLauncher = null;

    Servo servoGrabberLeft = null;
    Servo servoGrabberRight = null;
    Servo servoFinger = null;
    Servo servoLauncher = null;

    BNO055IMU imu;
    private BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    double headingOffset = 0.0;

    // Constants to be used in code. Measurements in millimeters
    private static final double GEAR_RATIO = 1.0; // Ratio of driven gear to driving gear
    private static final double TICKS_PER_MOTOR_REVOLUTION = 560.0;
    private static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    private static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    private static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;

    double slowModeDivisor = 1.0;

    enum ServoPositions
    {
        LAUNCHER_IDLE(0.0),
        FINGER_IDLE(0.0),
        FINGER_FIRING(1.0),
        GRABBER_RIGHT_STOW(0.0),
        GRABBER_RIGHT_RELEASE(1.0),
        GRABBER_LEFT_STOW(1.0),
        GRABBER_LEFT_RELEASE(0.0);

        public double pos;
        ServoPositions(double i)
        {
            pos = i;
        }
    }

    // Initialize hardware on robot
    void initHardware()
    {
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorLift = hardwareMap.dcMotor.get("motorLift");
        motorCollector = hardwareMap.dcMotor.get("motorCollector");
        motorLauncher = hardwareMap.dcMotor.get("motorLauncher");

        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        // Our drive motors seem to run at this speed
        motorFL.setMaxSpeed(2700);
        motorFR.setMaxSpeed(2700);
        motorBL.setMaxSpeed(2700);
        motorBR.setMaxSpeed(2700);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoGrabberRight = hardwareMap.servo.get("servoGrabberRight");
        servoGrabberLeft = hardwareMap.servo.get("servoGrabberLeft");
        servoFinger = hardwareMap.servo.get("servoFinger");
        servoLauncher = hardwareMap.servo.get("servoLauncher");

        servoGrabberRight.setPosition(ServoPositions.GRABBER_RIGHT_STOW.pos);
        servoGrabberLeft.setPosition(ServoPositions.GRABBER_LEFT_STOW.pos);
        servoFinger.setPosition(ServoPositions.FINGER_IDLE.pos);
        servoLauncher.setPosition(ServoPositions.LAUNCHER_IDLE.pos);

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.setMsTransmissionInterval(50);
    }

    // Sends information to Driver Station screen for drivers to see
    void sendTelemetry()
    {
        //TODO: Probably won't need this after testing. It takes up a lot of room, so remove if no longer needed.
        // Drive motor info
        telemetry.addData("FL Enc", formatNumber(motorFL.getCurrentPosition()));
        telemetry.addData("FR Enc", formatNumber(motorFR.getCurrentPosition()));
        telemetry.addData("BL Enc", formatNumber(motorBL.getCurrentPosition()));
        telemetry.addData("BR Enc", formatNumber(motorBR.getCurrentPosition()));

        telemetry.addData("FL", formatNumber(motorFL.getPower()));
        telemetry.addData("FR", formatNumber(motorFR.getPower()));
        telemetry.addData("BL", formatNumber(motorBL.getPower()));
        telemetry.addData("BR", formatNumber(motorBR.getPower()));

        telemetry.update();
    }

    // This allows the robot to drive in any direction and/or turn. Both autonomous and TeleOp use
    // this method, and may need to use some math. 0 degrees represents forward
    void driveMecanum(double driveAngle, double drivePower, double turnPower)
    {
        // A drive power over 1 doesn't make sense
        drivePower = Range.clip(drivePower, -1, 1);

        // Calculate x and y components of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
        double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
        double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        /*
         * Below is an explanation of how the mecanum wheels are driven
         *
         * Each wheel has a roller on the bottom mounted at 45 degrees. Because of this, each wheel
         * can only exert a force diagonally in one dimension. Using the front left wheel as an
         * example, when it is given a positive power, it exerts a force forward and right, which
         * means positive y and x. When the front right wheel is given a positive power, it exerts
         * a force forward and left, which means positive y and negative x. This is reflected in
         * how the motor powers are set. Turning is like standard tank drive.
         */
        // Set motor powers
        double powerFL = y + x - turnPower;
        double powerFR = y - x + turnPower;
        double powerBL = y - x - turnPower;
        double powerBR = y + x + turnPower;

        // Motor powers might be set above 1, so this scales all of the motor powers to stay
        // proportional and within power range
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Only apply scalar if greater than 1. Otherwise we could unintentionally increase power
        // This also prevents dividing by 0
        if(scalar < 1)
            scalar = 1;

        // Apply scalar
        powerFL /= (scalar * slowModeDivisor);
        powerFR /= (scalar * slowModeDivisor);
        powerBL /= (scalar * slowModeDivisor);
        powerBR /= (scalar * slowModeDivisor);

        // Set motor powers
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }

    void stopDriving()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    // Truncates numbers to fit displays better. Not recommended for numbers that span many
    // magnitudes. Also consider the decimal point character.
    private String formatNumber(double number)
    {
        return String.format("%.2f", number);
    }

    // Used for calculating distances between 2 points
    double calculateDistance(double deltaX, double deltaY)
    {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));
    }

    // Used to see if any buttons on the specified gamepad are currently pressed
    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide);
    }
}
