package org.firstinspires.ftc.team8923;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Set;

import javax.xml.transform.sax.TemplatesHandler;


/**
 * Runs the autonomous code to complete the 2017 summer beacon challenge
 */

@Autonomous(name = "Summer Beacon Challenge", group = "Summer")
public class SummerBeaconChallenge2017 extends LinearOpMode
{
    private DcMotor motorL;
    private DcMotor motorR;
    private DcMotor pusherL;
    private DcMotor pusherR;

    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorRight;
    private ColorSensor lightSensorLeft;
    private ColorSensor lightSensorRight;
    private BNO055IMU imu;

    private double TURN_CONSTANT = 1.0/1000.0; // somewhat arbitrary, will nail down in testing
    private double WHEEL_DIAMETER = 4.0; // inches
    private int lightTolerance = 5; // from 0 - 10
    private boolean start = false; // controls the exit from the pre-init setup loop
    private String color = "Red"; // denotes the color of the beacon to press
    private int distance; // distance traveled to find white line

    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        InitRobot();

        //SelectVariables();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {

            ManualDrive();

            /*turnToAngle(90);
            idle();
            wait(10000);*/

            /*
            //Drive until line
                driveUntilLine();
            //Line up with Beacon
                correctForBeacon();
            //Detect color and press button
                //detectAndPressBeaconButton();
            //Continue to next wall
                continueToWall();
            //Turn to angle
                turnToAngle(90);
            //Repeat
            */
        }
    }

    private void InitRobot()
    {
        motorL = hardwareMap.get(DcMotor.class, "motorL");
        motorR = hardwareMap.get(DcMotor.class, "motorR");
        //pusherL = hardwareMap.get(DcMotor.class, "pusherL");
        //pusherR = hardwareMap.get(DcMotor.class, "pusherR");

        /*colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        lightSensorLeft = hardwareMap.get(ColorSensor.class, "lightSensorLeft");
        lightSensorRight = hardwareMap.get(ColorSensor.class, "lightSensorRight");*/

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorL.setDirection(DcMotorSimple.Direction.REVERSE); //Motor is placed "backwards" so reversing it fixes normal power setting issues

        /*pusherL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pusherR.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/

        telemetry.addLine("starting IMU init");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addLine("finished IMU init");
        telemetry.update();
    }

    private void ManualDrive()
    {
        while(opModeIsActive())
        {
            motorL.setPower(gamepad1.left_stick_y / 2.0);
            motorR.setPower(gamepad1.right_stick_y / 2.0);
            displayTelemetry();
        }
    }

    private void SelectVariables()
    {
        //gamepad1.setJoystickDeadzone(0.15f);
        while(!start)
        {
            if(gamepad1.dpad_up && lightTolerance < 10)
                lightTolerance += 1;
            else if(gamepad1.dpad_down && lightTolerance > 0)
                lightTolerance -= 1;

            if(gamepad1.b)
                color = "Red";
            else if(gamepad1.x)
                color = "Blue";

            if(gamepad1.start)
                start = true;

            //prevent key repeat by waiting for all keys to be released here (also allows system process time)
            while(!buttonsAreReleased(gamepad1))
                idle();

            //display current settings to phone

            telemetry.addLine("-----------------------------");
            telemetry.addData("Light Tolerance (Up / Down)", lightTolerance);
            telemetry.addData("Color (X / B)", color);
            telemetry.addLine("Press \"Start\" to continue");
            telemetry.addLine("-----------------------------");
            telemetry.update();
        }
        telemetry.addLine("- - - Waiting for start - - -");
        telemetry.addLine("");
        telemetry.addLine("- - - Current Configuration - - -");
        telemetry.addData("Color", color);
        telemetry.addData("Light Tolerance", lightTolerance);
        telemetry.update();
    }

    private void displayTelemetry()
    {
        telemetry.addData("MotorL Power: ", motorL.getPower());
        telemetry.addData("LeftStick: ", gamepad1.left_stick_y);
        telemetry.addData("MotorR Power: ", motorR.getPower());
        telemetry.addData("RightStick: ", gamepad1.right_stick_y);
        telemetry.addData("Imu Angle: ", imu.getAngularOrientation().firstAngle);
        telemetry.update();
    }

    private void turnToAngle(double deltaAngle) throws InterruptedException
    {
        if(deltaAngle > 180 || deltaAngle < -180)
            throw new IllegalArgumentException("Delta Angle must be between -180 and 180 degrees");
        double ANGLE_TOLERANCE = 5.0;
        //TODO: update values to telemetry
        double currentAngle = imu.getAngularOrientation().firstAngle;
        double targetAngle = currentAngle + deltaAngle;

        while(Math.abs(currentAngle - targetAngle) > ANGLE_TOLERANCE)
        {
            double power = Range.clip((currentAngle - targetAngle) * TURN_CONSTANT, -1.0, 1.0);
            motorL.setPower(power);
            motorR.setPower(-1 * power);
        }
    }

    private void driveUntilLine()
    {
        double targetAngle = imu.getAngularOrientation().firstAngle;
        int initialEncoderValue = motorL.getCurrentPosition();
        while(GetBrightness(lightSensorLeft) < 5 && GetBrightness(lightSensorRight) < 5)
        {
            double correction = Range.clip((imu.getAngularOrientation().firstAngle - targetAngle) * TURN_CONSTANT, -0.35, 0.35);
            motorL.setPower(0.9 - correction);
            motorR.setPower(0.9 + correction);
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);
        distance = motorL.getCurrentPosition() - initialEncoderValue;
    }

    private void correctForBeacon()
    {
        double targetAngle = imu.getAngularOrientation().firstAngle;
        while(GetBrightness(lightSensorLeft) < 5 && GetBrightness(lightSensorRight) < 5)
        {
            double correction = Range.clip((imu.getAngularOrientation().firstAngle - targetAngle) * TURN_CONSTANT, -0.35, 0.35);
            motorL.setPower(-0.2 + correction);
            motorR.setPower(-0.2 - correction);
        }
        stopDriving();
    }

    private int GetBrightness (ColorSensor sensor)
    {
        return sensor.red() + sensor.blue() + sensor.green();
    }

    /*void detectAndPressBeaconButton() throws InterruptedException
    {
        int distance = 100;
        int[] colorsRight = new int[2];
        int[] colorsLeft = new int[2];
        colorsRight[0] = colorSensorRight.red();
        colorsRight[1] = colorSensorRight.blue();
        colorsLeft[0] = colorSensorLeft.red();
        colorsLeft[1] = colorSensorLeft.blue();

        if(colorsRight[0] > colorsLeft[0] && color == "Red")
            pushBeacon(motorR);
        else
            pushBeacon(motorL);
    }*/

    private void pushBeacon(DcMotor motor) throws InterruptedException
    {
        // in encoder ticks
        int travelDistance = 100;
        DcMotor.RunMode previousRunMode = DcMotor.RunMode.RUN_TO_POSITION;
        if(motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
        {
            previousRunMode = motor.getMode();
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        int currentEncoderPosition = motor.getCurrentPosition();
        int targetPosition = currentEncoderPosition + travelDistance;

        motor.setTargetPosition(targetPosition);
        motor.setPower(1.0);
        wait(100);
        motor.setTargetPosition(currentEncoderPosition);
        motor.setPower(-1.0);
        wait(500);
        motor.setPower(0.0);

        motor.setMode(previousRunMode);
    }

    private void continueToWall()
    {
        if(motorL.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(motorR.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int travelTicks = (int)Math.round(144 / (WHEEL_DIAMETER * Math.PI)) - distance;
        motorL.setTargetPosition(travelTicks);
        motorR.setTargetPosition(travelTicks);
        motorL.setPower(1.0);
        motorR.setPower(1.0);
        //TODO: replace with a check target method later
        while(motorL.isBusy() || motorR.isBusy())
        {
            telemetry.update();
            idle();
        }
        stopDriving();
    }

    private void stopDriving()
    {
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

    //wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t)
        {
            idle();
        }
    }

    boolean buttonsAreReleased(Gamepad pad)
    {
        return !(pad.a || pad.b || pad.x || pad.y || pad.left_bumper || pad.right_bumper
                || pad.dpad_up || pad.dpad_down || pad.dpad_left || pad.dpad_right
                || pad.left_stick_button || pad.right_stick_button
                || pad.start || pad.back || pad.guide || pad.left_trigger > 0.35
                || pad.right_trigger > 0.35);
    }

}
