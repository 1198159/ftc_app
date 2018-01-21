package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Test RR hardware to ensure functionality
 */

//@Disabled
@TeleOp(name = "RR Motor Test", group = "Test")
public class RRTest extends Master
{
    boolean going = true;
    boolean direction = true; //true is negative values added to ticks
    int RREncoderTicks = 0;
    ElapsedTime loopTimer = new ElapsedTime();
    ElapsedTime buttonTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException
    {
        InitHardware();
        waitForStart();
        motorRR.setTargetPosition(motorRR.getCurrentPosition());
        loopTimer.reset();
        while (opModeIsActive() && going)
        {
            if(gamepad1.a && buttonTimer.milliseconds() < 250)
            {
                buttonTimer.reset();
                direction = !direction;
            }
            loopTimer.reset();
            if(direction && RREncoderTicks > -840)
                RREncoderTicks--;
            else if(!direction && RREncoderTicks < 840)
                RREncoderTicks++;
            motorRR.setTargetPosition(RREncoderTicks);
            telemetry.addData("RR Ticks", RREncoderTicks);
            telemetry.addData("RR Motor Encoder", motorRR.getCurrentPosition());
            telemetry.update();
            motorRR.setPower(1.0);
            while (opModeIsActive() && !motorIsAtTarget(motorRR)) {idle();}
            motorRR.setPower(0.0);
            while (opModeIsActive() && loopTimer.milliseconds() > 100)
            {
                if(gamepad1.right_bumper)
                    going =  false;
            }
        }
    }
}
