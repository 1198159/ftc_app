package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();
        telemetry.addData("Init:", "Done");
        telemetry.update();
        waitForStart();
        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.b)
            {
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            omniDriveTeleOp();
            // TODO: add some telemetry to display the motor power
            idle();
        }
    }
}
