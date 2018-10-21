package org.firstinspires.ftc.team8923_2018;

import android.annotation.TargetApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Autonomous Competition", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetition extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //configureAutonomous();
        initAuto();

        waitForStart();

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        moveLift(500);
        /*
        motorFL.setPower(0.25);
        motorFR.setPower(0.25);
        motorBL.setPower(0.25);
        motorBR.setPower(0.25);
        */

        while (opModeIsActive())
        {
            telemetry.addData("FL power", motorFL.getPower());
            telemetry.addData("FR power", motorFR.getPower());
            telemetry.addData("BL power", motorBL.getPower());
            telemetry.addData("BR power", motorBR.getPower());
            telemetry.addData("FL encoder", motorFL.getCurrentPosition());
            telemetry.addData("FR encoder", motorFR.getCurrentPosition());
            telemetry.addData("BL encoder", motorBL.getCurrentPosition());
            telemetry.addData("BR encoder", motorBR.getCurrentPosition());
            telemetry.update();
            //driveToPoint(0,1000, 0, 1.0);
            idle();
        }

    }
}
