package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Linear Slide Test")
//@Disabled
public class LinearSlideTest extends MasterTeleOp
{
    int targetPos = 0;
    Servo rev1 = null; // hub 2 port 2

    public void runOpMode() throws InterruptedException
    {
        //super.initializeHardware();

        rev1 = hardwareMap.servo.get("rev1");
        rev1.setPosition(0.5);
        telemetry.addData("Init","done");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            rev1.setPosition(0.5);
            telemetry.addData("tarPos", targetPos);
            telemetry.update();
        }
    }
}
