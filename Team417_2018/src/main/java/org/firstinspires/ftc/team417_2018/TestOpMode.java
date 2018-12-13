package org.firstinspires.ftc.team417_2018;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "VexmotorTest")
//@Disabled
public class TestOpMode extends MasterAutonomous {

    public void runOpMode() throws InterruptedException
    {
        CRServo vex1 = null; // port 0
        vex1 = hardwareMap.crservo.get("vex1");

        telemetry.addData("Init:", "done");
        telemetry.update();
        waitForStart();

        vex1.setPower(0.0);

        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad2.b)
            {
                vex1.setPower(0.79);
            }
            else if (gamepad2.x)
            {
                vex1.setPower(-0.79);
            }
            else
            {
                vex1.setPower(0);
            }
            telemetry.update();
        }
    }
}
