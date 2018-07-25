package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Rev Hub Encoder Test Program")
public class REV_HUB_ENCODER_TESTING extends Master
{
    DcMotor motor;

    double encoderTicks = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime sleepTimer = new ElapsedTime();
        motor = hardwareMap.get(DcMotor.class, "motorTest");
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        timer.reset();
        while (opModeIsActive() && (timer.seconds() < 15))
        {
            int motorZero = motor.getCurrentPosition();
            motor.setTargetPosition(motorZero + 500);
            motor.setPower(0.25);
            while (motor.isBusy() && opModeIsActive())
            {
                telemetry.addData("expected", motorZero);
                telemetry.addData("actual", motor.getCurrentPosition());
                telemetry.addData("distance", Math.abs((motorZero + 500) - motor.getCurrentPosition()));
                telemetry.update();
            }
            sleepTimer.reset();
            while (sleepTimer.milliseconds() < 5000 && opModeIsActive())
                idle();
        }
        while (opModeIsActive())
            idle();
    }
}

