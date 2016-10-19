package org.firstinspires.ftc.team6220;

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
    //                                               mtr,                       x,   y,   rot,  gear, radius
    DriveAssembly[] driveAssemblies = { new DriveAssembly(null,new Transform2D( 1.0, 1.0, 135), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D(-1.0, 1.0, 225), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D(-1.0,-1.0, 315), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D( 1.0,-1.0,  45), 1.0, 0.1016) };

    DriveSystem drive;

    public void initializeHardware()
    {
        driveAssemblies[FRONT_RIGHT].motor = hardwareMap.dcMotor.get("motorFrontRight");
        driveAssemblies[FRONT_LEFT].motor  = hardwareMap.dcMotor.get("motorFrontLeft");
        driveAssemblies[BACK_RIGHT].motor  = hardwareMap.dcMotor.get("motorBackRight");
        driveAssemblies[BACK_LEFT].motor   = hardwareMap.dcMotor.get("motorBackLeft");


        //TODO decide if we should initialize at opmode level
        //                      drive assemblies,                  x , y,  w  ,               p  , i , d
        drive = new DriveSystem( driveAssemblies, new Transform2D(0.0,0.0,0.0), new PIDFilter(1.0,0.0,0.0) );

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
