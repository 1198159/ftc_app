package org.firstinspires.ftc.team8923;

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.xml.transform.sax.TemplatesHandler;


/**
 * Runs the autonomous code to complete the 2017 summer beacon challenge
 */

@Autonomous(name = "Summer Beacon Challenge", group = "Summer")
public class SummerBeaconChallenge2017 extends LinearOpMode
{
    DcMotor motorL;
    DcMotor motorR;
    DcMotor pusherL;
    DcMotor pusherR;

    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight;
    LightSensor lightSensorLeft;
    LightSensor lightSensorRight;
    LynxEmbeddedIMU imu;

    double TURN_CONSTANT = 1/100; // somewhat arbitrary, will nail down in testing
    double WHEEL_DIAMETER = 4.0; // inches
    double lightTolerance = 0.5; // from 0.0 - 1.0
    String color = "Red"; // [true = red / false = blue]
    int distance; // distance traveled to find white line

    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        SelectVariables();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            //DriveManual();

            turnToAngle(90);

            wait(10000);

            //Drive until line
                driveUntilLine();
            //Line up with Beacon
                correctForBeacon();
            //Detect color and press button
                detectAndPressBeaconButton();
            //Continue to next wall
                continueToWall();
            //Turn to angle
                turnToAngle(90);
            //Repeat
        }
    }

    public void InitRobot()
    {
        motorL = hardwareMap.get(DcMotor.class, "motorL");
        motorR = hardwareMap.get(DcMotor.class, "motorR");
        pusherL = hardwareMap.get(DcMotor.class, "pusherL");
        pusherR = hardwareMap.get(DcMotor.class, "pusherR");
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        lightSensorLeft = hardwareMap.get(LightSensor.class, "lightSensorLeft");
        lightSensorRight = hardwareMap.get(LightSensor.class, "lightSensorRight");
        imu = hardwareMap.get(LynxEmbeddedIMU.class, "imu");

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pusherL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        pusherR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void SelectVariables()
    {
        gamepad1.setJoystickDeadzone(0.15f);
        while(!gamepad1.a)
        {
            if(gamepad1.dpad_up)
                lightTolerance += 0.1;
            else if(gamepad1.dpad_down)
                lightTolerance -= 0.1;

            if(gamepad1.b)
                color = "Red";
            else if(gamepad1.x)
                color = "Blue";

            displayTelemetry();
        }
        InitRobot();
    }

    void displayTelemetry()
    {
        telemetry.clear();
        telemetry.addLine("-----------------------------");
        telemetry.addData("Light Tolerance:", lightTolerance);
        telemetry.addData("Color: ", color);
        telemetry.addLine("-----------------------------");
    }

    void turnToAngle(double deltaAngle) throws InterruptedException
    {
        if(deltaAngle > 180 || deltaAngle < -180)
            throw new IllegalArgumentException("Delta Angle must be between -180 and 180 degrees");
        double ANGLE_TOLERANCE = 5.0;
        double currentAngle = imu.getAngularOrientation().firstAngle;
        double targetAngle = currentAngle + deltaAngle;

        while(Math.abs(currentAngle - targetAngle) > ANGLE_TOLERANCE)
        {
            double power = Range.clip((currentAngle - targetAngle) * TURN_CONSTANT, -1.0, 1.0);
            motorL.setPower(power);
            motorR.setPower(-power);
        }
    }

    void driveUntilLine()
    {
        double targetAngle = imu.getAngularOrientation().firstAngle;
        int initialEncoderValue = motorL.getCurrentPosition();
        while(lightSensorLeft.getLightDetected() < 0.5 && lightSensorRight.getLightDetected() < 0.5)
        {
            double correction = Range.clip((imu.getAngularOrientation().firstAngle - targetAngle) * TURN_CONSTANT, -0.35, 0.35);
            motorL.setPower(0.9 - correction);
            motorR.setPower(0.9 + correction);
        }
        motorL.setPower(0.0);
        motorR.setPower(0.0);
        distance = motorL.getCurrentPosition() - initialEncoderValue;
    }

    void correctForBeacon()
    {
        double targetAngle = imu.getAngularOrientation().firstAngle;
        while(lightSensorLeft.getLightDetected() < 0.5 && lightSensorRight.getLightDetected() < 0.5)
        {
            double correction = Range.clip((imu.getAngularOrientation().firstAngle - targetAngle) * TURN_CONSTANT, -0.35, 0.35);
            motorL.setPower(-0.2 + correction);
            motorR.setPower(-0.2 - correction);
        }
        stopDriving();
    }

    void detectAndPressBeaconButton() throws InterruptedException
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
    }

    void pushBeacon(DcMotor motor) throws InterruptedException
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

    void continueToWall()
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

    void stopDriving()
    {
        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }

}
