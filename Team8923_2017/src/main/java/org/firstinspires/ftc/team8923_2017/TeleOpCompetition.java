package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.lang.reflect.GenericSignatureFormatError;

/**
 * Runable shell for Master TeleOp code
 */

@TeleOp(name = "CapBot Teleop")

public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        VoltageSensor voltSensor = hardwareMap.voltageSensor.get("Expansion Hub 1");
        InitHardware();


        waitForStart();

        while (opModeIsActive())
        {

            DriveOmni45TeleOp();
            RunGGLift();
            RunGGClaws();
            //double voltage = voltSensor.getVoltage();
            //telemetry.addData("Voltage", voltage);
            //telemetry.update();
            //RunGG();
            SendTelemetry();
            idle();
        }
    }
}

