package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 program designed for the beacon run challenge
 */

@Autonomous(name = "BeaconRunAuto", group = "Autonomous")
public class BeaconRunAuto extends LinearOpMode
{
    //create hardware devices
    private DcMotor motorFL;
    private DcMotor motorFR;
    private DcMotor motorBL;
    private DcMotor motorBR;
    private Servo beaconServo;
    private ColorSensor colorSensorL;
    private ColorSensor colorSensorR;
    //todo:  light sensors won't work yet!
    private LightSensor lightSensorF;
    private LightSensor lightSensorB;
    private BNO055IMU imu;

    private int EncoderFL = 0;
    private int EncoderFR = 0;
    private int EncoderBL = 0;
    private int EncoderBR = 0;

    private double currentAngle = 0.0;
    double angleTolerance = 2.0;

    //retrieve and initialize hardware devices
    public void initializeHardware()
    {
        motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFR = hardwareMap.dcMotor.get("motorFrontRight");
        motorBL = hardwareMap.dcMotor.get("motorBackLeft");
        motorBR = hardwareMap.dcMotor.get("motorBackRight");
        beaconServo = hardwareMap.servo.get("beaconServo");
        colorSensorL = hardwareMap.colorSensor.get("colorSensorL");
        colorSensorR = hardwareMap.colorSensor.get("colorSensorR");
        lightSensorF = hardwareMap.lightSensor.get("lightSensorF");
        lightSensorB = hardwareMap.lightSensor.get("lightSensorB");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        //CodeReview: Did we include the calibration json somewhere so it can be found in our program?
        //            If we are going to reference this file, it has to exist, and has to be where the
        //            calibration sample opmode puts it (so it can be found)
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
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

    //prevents angle differences from being out of range
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        while (Math.abs(diff) > 180)
        {
            diff -= Math.signum(diff) * 360;
        }

        return diff;
    }

    public void turnTo(double targetAngle)
    {
        while(Math.abs(currentAngle - targetAngle) > angleTolerance)
        {
            //todo: check if a constant is needed here
            double turningPower = currentAngle - targetAngle;

            //ensure turn power doesn't dip below minimum power
            if(turningPower > 0 && turningPower < Constants.MINIMUM_TURNING_POWER)
                turningPower = Constants.MINIMUM_TURNING_POWER;
            else if(turningPower < 0 && turningPower > -Constants.MINIMUM_TURNING_POWER)
                turningPower = -Constants.MINIMUM_TURNING_POWER;

            motorBL.setPower(turningPower);
            motorFL.setPower(turningPower);
            motorBR.setPower(turningPower);
            motorFR.setPower(turningPower);
        }
    }

    //robot drives straight until white line; utilizes light sensors
    public void driveToLine()
    {

    }

    //utilizes color sensors to decide which side of beacon to press
    public void activateBeacon() throws InterruptedException
    {
        double leftBeaconRedValue = colorSensorL.red();
        double rightBeaconRedValue = colorSensorR.red();

        //todo: determine positioning of servo and adjust input values
        //compare values and move servo
        if (leftBeaconRedValue > rightBeaconRedValue)
        {
            beaconServo.setPosition(1.0);
            pause(300);
            beaconServo.setPosition(0.0);
        }
        else
        {
            beaconServo.setPosition(-1.0);
            pause(300);
            beaconServo.setPosition(0.0);
        }
    }

    public void runOpMode() throws InterruptedException
    {
        initializeHardware();


    }
}
