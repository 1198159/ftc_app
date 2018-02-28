package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
        telemetry.addLine("Hardware Init Started");
        telemetry.update();
        InitHardware();
        telemetry.addLine("Hardware Init Finished");
        telemetry.update();

        waitForStart();
        // Set servo positions so no penalty
        servoGGUL.setPosition(GGServoPositions.LEFTFULLOPEN.val()); //TODO value needs to be changed
        servoGGUR.setPosition(GGServoPositions.RIGHTFULLOPEN.val()); //TODO value needs to be changed
        servoGGDL.setPosition(GGServoPositions.LEFTFULLOPEN.val()); //TODO value needs to be changed
        servoGGDR.setPosition(GGServoPositions.RIGHTFULLOPEN.val()); //TODO value needs to be changed
        servoJJ.setPosition(SERVO_JJ_UP);

        while (opModeIsActive())
        {
            AutoBalance();
            if(!autoBalancing)
            {
                DriveOmni45TeleOp();
                RunGGLift();
                RunGGClaws();
                //RunRR();
            }
            SendTelemetry();
            idle();
        }
    }
}

