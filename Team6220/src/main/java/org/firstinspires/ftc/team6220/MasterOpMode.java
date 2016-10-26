package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



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


    //our robot uses an omni drive, so our motors are positioned at 45 degree angles to normal positions.
    //TODO update drive assembly positions
    //
    DriveAssembly[] driveAssemblies;

    DriveSystem drive;

    VuforiaHelper vuforiaHelper;

    public void initializeHardware()
    {

        driveAssemblies = new DriveAssembly[4];
                                                                        //mtr,                                       x,   y,   rot,  gear, radius
        driveAssemblies[FRONT_RIGHT] = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontRight"),new Transform2D( 1.0, 1.0, 135), 1.0, 0.1016);
        driveAssemblies[FRONT_LEFT] = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontLeft"),new Transform2D(-1.0, 1.0, 225), 1.0, 0.1016);
        driveAssemblies[BACK_LEFT] = new DriveAssembly(hardwareMap.dcMotor.get("motorBackLeft"),new Transform2D(-1.0,-1.0, 315), 1.0, 0.1016);
        driveAssemblies[BACK_RIGHT] = new DriveAssembly(hardwareMap.dcMotor.get("motorBackRight"),new Transform2D( 1.0,-1.0,  45), 1.0, 0.1016);

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

        // Inform drivers of robot location. Location is null if we lose track of targets
        if(vuforiaHelper.lastKnownLocation != null)
            telemetry.addData("Pos", vuforiaHelper.format(vuforiaHelper.lastKnownLocation));
        else
            telemetry.addData("Pos", "Unknown");

        telemetry.update();


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
