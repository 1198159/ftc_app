package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        super.initializeHardware();
        core2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // set encoder value to 0
        core2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Init:", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            mecanumDrive();
            linearSlides();
            collector();
            updateTelemetry();
        }
    }
}
