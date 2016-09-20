package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



/*

*/

abstract public class MasterOpMode extends LinearOpMode
{
    public final int FRONT_RIGHT = 0;
    public final int FRONT_LEFT = 0;
    public final int BACK_LEFT = 0;
    public final int BACK_RIGHT = 0;


    //our robot uses an omni drive, so our motors are positioned at 45 degree angles to normal positions.
    //TODO update drive assembly positions
    //                                                    mtr,                  x,   y,   rot,  gear, radius
    DriveAssembly[] driveAssem = { new DriveAssembly(null,new Transform2D( 1.0, 1.0, 135), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D(-1.0, 1.0, 225), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D(-1.0,-1.0, 315), 1.0, 0.1016),
                                        new DriveAssembly(null,new Transform2D( 1.0,-1.0,  45), 1.0, 0.1016) };

    public void initializeHardware()
    {
        driveAssem[FRONT_RIGHT].motor = hardwareMap.dcMotor.get("motorFrontRight");
        driveAssem[FRONT_LEFT] .motor = hardwareMap.dcMotor.get("motorFrontLeft");
        driveAssem[BACK_LEFT]  .motor = hardwareMap.dcMotor.get("motorBackRight");
        driveAssem[BACK_RIGHT] .motor = hardwareMap.dcMotor.get("motorBackLeft");
    }
}
