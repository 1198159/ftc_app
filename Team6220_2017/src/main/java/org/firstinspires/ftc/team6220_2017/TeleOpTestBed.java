package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * A test program used to try out any new ideas.
 */
@TeleOp(name="Test Bed Program", group = "6220")
//@Disabled
public class TeleOpTestBed extends MasterOpMode
{
    CRServo rightServo;
    CRServo leftServo;
    Servo servo;
    VuforiaHelper vuforiaHelper = new VuforiaHelper();

    @Override public void runOpMode() throws InterruptedException
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        //telemetry.log().setCapacity(4);
        /*rightServo = hardwareMap.crservo.get("servoRight");
        leftServo = hardwareMap.crservo.get("servoLeft");*/
        //servo = hardwareMap.servo.get("servo");

        vuforiaHelper.setupVuforia();
        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            if(gamepad1.x)
            {
                vuforiaHelper.getJewelColor();
                //output red hue values of both sides for debugging
                telemetry.addData("LeftSideHue: ", vuforiaHelper.leftColorOutput[0]);
                telemetry.addData("RightSideHue: ", vuforiaHelper.rightColorOutput[0]);
                telemetry.update();
            }

            //for testing glyph collection; servos on either side of glyph pull or push it
            if(gamepad1.a)
            {
                //rightServo.setPower(1.0);
                //leftServo.setPower(-1.0);
            }
            if(gamepad1.b)
            {
                //rightServo.setPower(-1.0);
                //leftServo.setPower(1.0);
            }

            //for robot arm test
            //servo.setPosition((gamepad1.left_stick_y + 1) / 2);

            idle();
        }
    }

}
