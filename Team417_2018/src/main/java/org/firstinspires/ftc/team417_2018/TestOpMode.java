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

    CRServo vex1 = null; // port 0
    Servo rev1 = null;

    public void runOpMode() throws InterruptedException
    {


        vex1 = hardwareMap.crservo.get("vex1");
        rev1 = hardwareMap.servo.get("rev1");

        telemetry.addData("Init:", "done");
        telemetry.update();
        waitForStart();

        vex1.setPower(0.0);
        rev1.setPosition(0.0);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad2.b)
            {
                vex1.setPower(0.79);
            }
            else if (gamepad2.a)
            {
                vex1.setPower(-0.79);
            }
            else
            {
                vex1.setPower(0);

            }

        }
    }
}
