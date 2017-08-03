package org.firstinspires.ftc.teamswerve;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.Locale;


/**
 * Program used to control Drive-A-Bots.YAY!!
 * This can be a good reference for drive controls.
 */
@TeleOp(name="JustinOmniDrive", group = "Swerve")
// @Disabled
public class JustinOmniDrive extends LinearOpMode
{
    DcMotor motorFront = null;
    DcMotor motorBack = null;
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    double currentAngle;
    double startAngle;
    double imuAngle;
    // double currentAngleY;
    // double currentAngleZ;
    Orientation angles;
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // Drive mode constants

    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            //angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
            //currentAngle = angles.firstAngle - startAngle;

            imuAngle = imu.getAngularOrientation().firstAngle;
            currentAngle = imuAngle - startAngle;

            // currentAngleY = angles.firstAngle - startAngle;
            // currentAngleZ = angles.firstAngle - startAngle;

            // Gamepads have a new state, so update things that need updating
            //if(updateGamepads())
            {
                // Set drive mode

                // Run drive mode
                omniDrive();
            }

            telemetry.update();
            idle();
        }
    }

    /*
     * Controls the robot with two joysticks
     * Left joystick controls left side
     * Right joystick controls right side
     */
    public void omniDrive()
    {
        double frontPower;
        double backPower;
        double leftPower;
        double rightPower;
        double jx;
        double jy;
        double jt;
        double jp;
        jt = gamepad1. left_stick_x;
        jp = -gamepad1.left_stick_x;
        jx = gamepad1.right_stick_x;
        jy = -gamepad1.right_stick_y;


        motorFront.setPower(jx);
        motorBack.setPower(jx);
        motorLeft.setPower(jy);
        motorRight.setPower(jy);
        motorFront.setPower(jt);
        motorBack.setPower(jp);
        motorLeft.setPower(jt);
        motorRight.setPower(jp);
    }

    /*
     * Controls the robot with a single joystick
     * Forward, backward, right, and left on joystick control
     *
     */
    public void arcadeDrive()
    {
        double forwardPower = gamepad1.left_stick_y;
        double turningPower = Math.pow(Math.abs(gamepad1.left_stick_x), 2) * Math.signum(gamepad1.left_stick_x);
        double leftPower = forwardPower - turningPower;
        double rightPower = forwardPower + turningPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    /*
     * Controls robot like a racing video game
     * Right trigger moves robot forward
     * Left trigger moves robot backward
     * Left stick for turning
     */
    public void gameDrive()
    {
        double forwardPower = gamepad1.left_trigger - gamepad1.right_trigger;
        double turningPower = Math.pow(gamepad1.left_stick_x, 2) * Math.signum(gamepad1.left_stick_x); // This multiplier is because the robot turns too quickly

        double leftPower = forwardPower - turningPower;
        double rightPower = forwardPower + turningPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    /*
     * Arcade drive with 2 joysticks
     */
    public void splitArcadeDrive()
    {
        double forwardPower = gamepad1.left_stick_y;
        double turningPower = Math.pow(Math.abs(gamepad1.right_stick_x), 2) * Math.signum(gamepad1.right_stick_x);

        double leftPower = forwardPower - turningPower;
        double rightPower = forwardPower + turningPower;

        motorLeft.setPower(leftPower);
        motorRight.setPower(rightPower);
    }

    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        motorFront = hardwareMap.dcMotor.get("motorFront");
        motorBack = hardwareMap.dcMotor.get("motorBack");
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        // We're not using encoders, so tell the motor controller
        motorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The motors will run in opposite directions, so flip one
        //THIS IS SET UP FOR TANK MODE WITH OUR CURRENT DRIVABOTS
        //DON'T CHANGE IT!
        motorFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up telemetry data
        configureDashboard();
    }

    public void configureDashboard()
    {
        telemetry.addLine()
                .addData("Power | Front: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorFront.getPower());
                    }
                })
                .addData("Back: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorBack.getPower());
                    }
                })
                .addData("Power | Left: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorLeft.getPower());
                    }
                })
                .addData("Right: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motorRight.getPower());
                    }
                })

                .addData("heading", new Func<String>() {
                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, currentAngle);
                        return formatNumber(currentAngle);

                    }
                })

                .addData("startAngle", new Func<String>() {
                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, startAngle);
                        return formatNumber(startAngle);
                    }
                })

                .addData("imuAngle", new Func<String>() {
                    @Override public String value() {
//                        return formatAngle(angles.angleUnit, imuAngle);
                        return formatNumber(imuAngle);
                    }
                });

    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
}