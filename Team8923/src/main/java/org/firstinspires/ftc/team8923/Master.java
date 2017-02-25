package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    DcMotor motorCatapult = null;

    Servo servoBeaconPusherDeploy = null;
    Servo servoBeaconPusherSwing = null;
    Servo servoCapBallHolder = null;
    Servo servoHopperSweeper = null;
    Servo servoCollectorHolder = null;

    TouchSensor catapultButton;

    double headingOffset = 0.0;

    // Also used as the arming location of the catapult
    int catapultZero = 0;

    // Constants to be used in code. Measurements in millimeters
    private static final double GEAR_RATIO = 1.5; // Ratio of driven gear to driving gear
    private static final double TICKS_PER_MOTOR_REVOLUTION = 1120.0;
    private static final double TICKS_PER_WHEEL_REVOLUTION = TICKS_PER_MOTOR_REVOLUTION / GEAR_RATIO;
    private static final double WHEEL_DIAMETER = 4 * 25.4; // 4 inch diameter
    private static final double MM_PER_REVOLUTION = Math.PI * WHEEL_DIAMETER;
    static final double MM_PER_TICK = MM_PER_REVOLUTION / TICKS_PER_WHEEL_REVOLUTION;

    // NeveRest 20 has 560 ticks per revolution, which is geared 3:1
    static final int CATAPULT_TICKS_PER_CYCLE = 560 * 3;

    double slowModeDivisor = 1.0;
    private boolean reverseDrive = false;

    enum ServoPositions
    {
        BEACON_RETRACT(0.80),
        BEACON_EXTEND(0.15),
        BEACON_RIGHT(0.05),
        BEACON_LEFT(0.7),
        BEACON_CENTER(0.5),
        CAP_BALL_HOLD(1.0),
        CAP_BALL_RELEASE(0.0),
        HOPPER_SWEEP_BACK(0.15),
        HOPPER_SWEEP_PUSH_FIRST(0.7),
        HOPPER_SWEEP_PUSH_SECOND(1.0),
        COLLECTOR_HOLDER_DOWN(0.65),
        COLLECTOR_HOLDER_UP(0.0);

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
        motorCatapult = hardwareMap.dcMotor.get("motorCatapult");

        // Set drive motor directions
        reverseDrive(false);
        // Some hardware person got the wiring backwards...
        motorLift.setDirection(DcMotor.Direction.FORWARD); // changed due to chain replacing our old gear system for driving the lift
        motorCollector.setDirection(DcMotor.Direction.REVERSE);

        // Our drive motors seem to run at this speed
        motorFL.setMaxSpeed(2700);
        motorFR.setMaxSpeed(2700);
        motorBL.setMaxSpeed(2700);
        motorBR.setMaxSpeed(2700);
        motorCatapult.setMaxSpeed(2700);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorCatapult.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorCatapult.setDirection(DcMotor.Direction.FORWARD);

        servoBeaconPusherDeploy = hardwareMap.servo.get("servoBeaconPusherDeploy");
        servoBeaconPusherSwing = hardwareMap.servo.get("servoBeaconPusherSwing");
        servoCapBallHolder = hardwareMap.servo.get("servoCapBallHolder");
        servoHopperSweeper = hardwareMap.servo.get("servoHopperSweeper");
        servoCollectorHolder = hardwareMap.servo.get("servoCollectorHolder");

        servoBeaconPusherDeploy.setPosition(ServoPositions.BEACON_RETRACT.pos);
        servoBeaconPusherSwing.setPosition(ServoPositions.BEACON_CENTER.pos);
        servoCapBallHolder.setPosition(ServoPositions.CAP_BALL_RELEASE.pos);
        servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
        servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_DOWN.pos);


        catapultButton = hardwareMap.touchSensor.get("catapultButton");

        // Drivers need to get data quickly, and this doesn't take up too much bandwidth
        telemetry.setMsTransmissionInterval(50);
    }

    void reverseDrive(boolean reverse)
    {
        // Normally one side of the robot would be reversed, but because the front motors use gears
        // and the back motors use chain, opposite corners are reversed

        // Forwards
        if(!reverse)
        {
            reverseDrive = false;
            motorFL.setDirection(DcMotor.Direction.FORWARD);
            motorFR.setDirection(DcMotor.Direction.REVERSE);
            motorBL.setDirection(DcMotor.Direction.FORWARD);
            motorBR.setDirection(DcMotor.Direction.REVERSE);
            return;
        }

        // Reverse
        reverseDrive = true;
        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.FORWARD);
        motorBL.setDirection(DcMotor.Direction.REVERSE);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
    }

    // Sends information to Driver Station screen for drivers to see
    void sendTelemetry()
    {
        telemetry.addData("Catapult Encoder", motorCatapult.getCurrentPosition());
        telemetry.addData("Catapult Target", motorCatapult.getTargetPosition());
        telemetry.addData("Catapult Power", motorCatapult.getPower());
        telemetry.addData("Catapult Busy", motorCatapult.isBusy());
        telemetry.addData("Catapult Button", catapultButton.isPressed());

        telemetry.addData("Lift Encoder", motorLift.getCurrentPosition());

        // Drive motor info
        telemetry.addData("Reversed", reverseDrive);

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
        if(!reverseDrive)
        {
            // Drive forwards
            motorFL.setPower(powerFL);
            motorFR.setPower(powerFR);
            motorBL.setPower(powerBL);
            motorBR.setPower(powerBR);
            return;
        }

        // Drive backwards
        motorFL.setPower(powerBR);
        motorFR.setPower(powerBL);
        motorBL.setPower(powerFR);
        motorBR.setPower(powerFL);
    }

    void stopDriving()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    // This is used as a replacement for the isBusy() method of motors, as it's unreliable
    boolean motorIsAtTarget(DcMotor motor)
    {
        int tolerance = 100;
        return Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < tolerance;
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
