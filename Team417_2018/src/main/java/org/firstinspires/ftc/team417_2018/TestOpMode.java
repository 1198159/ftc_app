package org.firstinspires.ftc.team417_2018;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TestOpMode")
//@Disabled
public class TestOpMode extends LinearOpMode
{

    DcMotor core1 = null; // hub 2 port 2

    public void runOpMode() throws InterruptedException
    {
        core1 = hardwareMap.dcMotor.get("core1");

        telemetry.addData("Init:", "done");
        telemetry.update();
        waitForStart();

        core1.setPower(0.0);

        waitForStart();

        while (opModeIsActive())
        {
            core1.setPower(-gamepad1.left_stick_y); // if left stick up then slides extend
        }
    }
}
