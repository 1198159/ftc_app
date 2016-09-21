package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/*

*/

abstract public class MasterOpMode extends LinearOpMode
{
    //Java Enums cannot act as integer indices as they do in other languages
    //To maintain compatabliity with with a FOR loop, we
    //TODO find is java supports a order-independent loop for each member of an array
    //e.g. in python that would be:   for assembly in driveAssemblies:
    //                                    assembly.doStuff()
    public final int FRONT_RIGHT = 0;
    public final int FRONT_LEFT = 0;
    public final int BACK_LEFT = 0;
    public final int BACK_RIGHT = 0;


    //our robot uses an omni drive, so our motors are positioned at 45 degree angles to normal positions.
    //TODO update drive assembly positions
    //                                               mtr,                  x,   y,   rot,  gear, radius
    DriveAssembly[] driveAssemblies = { new DriveAssembly(null,new Transform2D( 1.0, 1.0, 135), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D(-1.0, 1.0, 225), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D(-1.0,-1.0, 315), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D( 1.0,-1.0,  45), 1.0, 0.1016) };

    DriveSystem drive;

    public void initializeHardware()
    {
        driveAssemblies[FRONT_RIGHT].motor = hardwareMap.dcMotor.get("motorFrontRight");
        driveAssemblies[FRONT_LEFT] .motor = hardwareMap.dcMotor.get("motorFrontLeft");
        driveAssemblies[BACK_LEFT]  .motor = hardwareMap.dcMotor.get("motorBackRight");
        driveAssemblies[BACK_RIGHT] .motor = hardwareMap.dcMotor.get("motorBackLeft");

        //                     drive assemblies,                  x , y,  w  ,               p  , i , d  ,      mode
        drive = new DriveSystem(driveAssemblies, new Transform2D(0.0,0.0,0.0), new PIDFilter(1.0,0.0,0.0), PIDEnforcementMode.NONE);
        
    }
}
