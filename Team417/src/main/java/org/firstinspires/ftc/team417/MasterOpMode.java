package org.firstinspires.ftc.team417;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by user_2 on 9/20/2016.
 */
abstract public class MasterOpMode extends LinearOpMode
{
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    public void initializeHardware()
    {
        //motorFrontLeft = hardwareMap.dcMotor.get
    }
}
