package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



/*

*/

abstract public class MasterOpMode extends LinearOpMode
{
    //Java Enums cannot act as integer indices as they do in other languages
    //To maintain compatabliity with with a FOR loop, we
    //TODO find if java supports an order-independent loop for each member of an array
    //e.g. in python that would be:   for assembly in driveAssemblies:
    //                                    assembly.doStuff()
    public final int FRONT_RIGHT = 0;
    public final int FRONT_LEFT  = 1;
    public final int BACK_LEFT   = 2;
    public final int BACK_RIGHT  = 3;

    private int EncoderFR = 0;
    private int EncoderFL = 0;
    private int EncoderBL = 0;
    private int EncoderBR = 0;

    double robotXPos = 0;
    double robotYPos = 0;

    //TODO deal with angles at all starting positions
    double currentAngle = 0;

    BNO055IMU imu;

    DriveAssembly[] driveAssemblies;

    DriveSystem drive;

    VuforiaHelper vuforiaHelper;

    public void initializeHardware()
    {
        //create a driveAssembly array to allow for easy access to motors
        driveAssemblies = new DriveAssembly[4];

        //our robot uses an omni drive, so our motors are positioned at 45 degree angles to motor positions on a normal drive.
                                                                        //mtr,                                       x,   y,   rot,  gear, radius
        driveAssemblies[FRONT_RIGHT] = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontRight"),new Transform2D( 1.0, 1.0, 135), 1.0, 0.1016);
        driveAssemblies[FRONT_LEFT] = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontLeft"),new Transform2D(-1.0, 1.0, 225), 1.0, 0.1016);
        driveAssemblies[BACK_LEFT] = new DriveAssembly(hardwareMap.dcMotor.get("motorBackLeft"),new Transform2D(-1.0,-1.0, 315), 1.0, 0.1016);
        driveAssemblies[BACK_RIGHT] = new DriveAssembly(hardwareMap.dcMotor.get("motorBackRight"),new Transform2D( 1.0,-1.0,  45), 1.0, 0.1016);

        driveAssemblies[FRONT_RIGHT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[FRONT_LEFT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[BACK_LEFT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[BACK_RIGHT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //TODO decide if we should initialize at opmode level
        //                      drive assemblies,                  x , y,  w  ,               p  , i , d
        drive = new DriveSystem( driveAssemblies, new Transform2D(0.0,0.0,0.0), new PIDFilter(1.0,0.0,0.0) );

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        vuforiaHelper = new VuforiaHelper();

        vuforiaHelper.setupVuforia();
    }

    public void navigateRed1()
    {
        // Start tracking targets
        vuforiaHelper.visionTargets.activate();

        vuforiaHelper.lastKnownLocation = vuforiaHelper.getLatestLocation();
        //updateLocation()

        // Inform drivers of robot location. Location is null if we lose track of targets
        if(vuforiaHelper.lastKnownLocation != null)
            telemetry.addData("Pos", vuforiaHelper.format(vuforiaHelper.lastKnownLocation));
        else
            telemetry.addData("Pos", "Unknown");

        telemetry.update();


    }

    //TODO check encoder and imu loocation function
    //keeps track of the robot's location in the arena based on Encoders and IMU
    public void updateLocation()
    {
        currentAngle = imu.getAngularOrientation().firstAngle;

        //find encoder values for motors
        EncoderFR = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderFL = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderBL = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderBR = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();

        //math to calculate x and y positions based on encoder ticks and robot angle
        robotXPos = Math.cos(currentAngle) * (EncoderFR + EncoderFL) / Math.pow(2, 0.5);
        robotYPos = Math.sin(currentAngle) * (EncoderFR + EncoderFL) / Math.pow(2, 0.5);

        telemetry.addData("X:", robotXPos);
        telemetry.addData("Y:", robotYPos);
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
}
