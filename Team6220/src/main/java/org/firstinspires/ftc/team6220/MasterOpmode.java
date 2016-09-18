package org.firstinspires.ftc.team6220;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Colew on 9/18/2016.
 */
abstract public class MasterOpmode extends LinearOpMode
{
    //our robot uses an omni drive, so our motors are positioned at 45 degree angles to normal positions.
    DcMotor motorTopRight;
    DcMotor motorTopLeft;
    DcMotor motorBottomRight;
    DcMotor motorBottomLeft;

    public void initializeHardware()
    {
        motorTopRight = hardwareMap.dcMotor.get("motorTopRight");
        motorTopLeft = hardwareMap.dcMotor.get("motorTopLeft");
        motorBottomRight = hardwareMap.dcMotor.get("motorBottomRight");
        motorBottomLeft = hardwareMap.dcMotor.get("motorBottomLeft");
    }
}
