package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.lang.reflect.GenericSignatureFormatError;

/**
 * Runable shell for Master TeleOp code
 */

@TeleOp(name = "CapBot TeleOp")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        InitHardware();


        waitForStart();
        // Set servo positions so no penalty
        servoGGL.setPosition(0.65);
        servoGGR.setPosition(0.22);
        servoJJ.setPosition(SERVO_JJ_UP);



        while (opModeIsActive())
        {

            DriveOmni45TeleOp();
            RunGGLift();
            RunGGClaws();
            //RunRR();
            SendTelemetry();
            idle();
        }
    }
}

