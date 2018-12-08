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

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Init:", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())

            mecanumDrive();
            collector();
        if (gamepad2.right_trigger > 0) // extend the collector
        {
            core1Power = -gamepad2.right_trigger;
            core2Power = -gamepad2.right_trigger;

        }
        else if (gamepad2.left_trigger > 0) // pull the collector in
        {
            core1Power = gamepad2.left_trigger;
            core2Power = gamepad2.left_trigger;
        }
        if (gamepad2.dpad_left)
        {
            core1.setPower(Range.clip(core1Power, -ADAGIO_POWER, ADAGIO_POWER));
            core2.setPower(Range.clip(core1Power, -ADAGIO_POWER, ADAGIO_POWER));
        }
        else
        {
            core1Power = 0.0;
            core2Power = 0.0;
        }
        core1.setPower(core1Power);
        core2.setPower(core2Power);

        // control AM 3.7 motors
        if (gamepad2.right_stick_y != 0)
        {
            arm1.setPower(gamepad2.right_stick_y);
            arm2.setPower(-gamepad2.right_stick_y);
        }
        else if (gamepad2.right_stick_y != 0 && gamepad2.dpad_left)
        {
            arm1.setPower(Range.clip(gamepad2.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER));
            arm2.setPower(Range.clip(-gamepad2.right_stick_y, -ADAGIO_POWER, ADAGIO_POWER));
        }
        else
        {
            arm1.setPower(0.0);
            arm2.setPower(0.0);
        }

        if (gamepad2.left_bumper)
        {
            hanger.setPower(0.99);
        }
        else if (gamepad2.right_bumper)
        {
            hanger.setPower(-0.99);
        }
        else
        {
            hanger.setPower(0.0);
        }
            marker();
            updateTelemetry();
            idle();
        }
    }
