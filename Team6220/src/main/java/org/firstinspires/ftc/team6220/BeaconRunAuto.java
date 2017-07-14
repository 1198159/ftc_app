package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;

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
    private ColorSensor colorLeft;
    private ColorSensor colorRight;
    //todo:  light sensors won't work yet!
    private LightSensor lightFront;
    private LightSensor lightBack;
    private BNO055IMU imu;

    //retrieve and initialize hardware devices
    public void initializeHardware()
    {
        motorFL = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFR = hardwareMap.dcMotor.get("motorFrontRight");
        motorBL = hardwareMap.dcMotor.get("motorBackLeft");
        motorBR = hardwareMap.dcMotor.get("motorBackRight");
        colorLeft = hardwareMap.colorSensor.get("colorSensorLeft");
        colorRight = hardwareMap.colorSensor.get("colorSensorRight");
        lightFront = hardwareMap.lightSensor.get("lightSensorFront");
        lightBack = hardwareMap.lightSensor.get("lightSensorBack");
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

    //robot drives straight until white line; utilizes light sensors
    public void driveToLine()
    {

    }

    //utilizes color sensors to decide which side of beacon to press
    public void identifyBeaconColor()
    {

    }

    //uses color determination to press beacon
    public void activateBeacon()
    {

    }



    public void runOpMode() throws InterruptedException
    {
        initializeHardware();


    }
}
