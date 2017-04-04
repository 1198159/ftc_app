package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Move Test", group = "Tests")
public class MoveTest extends MasterTeleOp
{
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        waitForStart();

        timer.reset();
        while(timer.milliseconds() < 5000)
        {
            motorFL.setPower(1.0);
            motorFR.setPower(-1.0);
            motorBL.setPower(-1.0);
            motorBR.setPower(1.0);
        }
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }
}
