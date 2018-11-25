package org.firstinspires.ftc.team417_2018;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="TestServos", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class HelloWorld extends LinearOpMode
{

    CRServo vex1 = null;

    public void runOpMode()
    {
        vex1 = hardwareMap.crservo.get("vex1");
        telemetry.addData("init", "done");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        vex1.setPower(0.5);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
           if(gamepad1.a)
           {
               vex1.setPower(0.79);
           }
           else if (gamepad1.b)
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