package org.firstinspires.ftc.team417_2018;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name = "Reset")
//@Disabled
public class TestOpMode extends MasterAutonomous
{
    public void runOpMode() throws InterruptedException
    {

        autoInitializeRobot();

        telemetry.addData("Init:", "done");
        telemetry.update();
        waitForStart();

        reset();
    }
}
