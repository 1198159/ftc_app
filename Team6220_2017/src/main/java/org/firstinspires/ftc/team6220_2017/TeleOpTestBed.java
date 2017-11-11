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
public class TeleOpTestBed extends MasterAutonomous
{
    CRServo rightServo;
    CRServo leftServo;
    Servo servo;
    VuforiaHelper vuforiaHelper = new VuforiaHelper();

    @Override public void runOpMode() throws InterruptedException
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        //initializeHardware();

        DcMotor glyphMotorLeft;
        DcMotor glyphMotorRight;

        glyphMotorLeft = hardwareMap.dcMotor.get("glyphMotorLeft");
        glyphMotorRight = hardwareMap.dcMotor.get("glyphMotorRight");

        vuforiaHelper.setupVuforia();
        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            //for motor that actuates arm
            motorArm.setPower(gamepad1.left_stick_y);

            /*
            if(gamepad1.x)
            {
                vuforiaHelper.getJewelColor(this);
                //output red hue values of both sides for debugging
                telemetry.addData("LeftSideHue: ", vuforiaHelper.leftColorOutput[0]);
                telemetry.addData("RightSideHue: ", vuforiaHelper.rightColorOutput[0]);
                telemetry.addData("R: ", vuforiaHelper.colorOutput[0]);
                telemetry.addData("G: ", vuforiaHelper.colorOutput[1]);
                telemetry.addData("B: ", vuforiaHelper.colorOutput[2]);
                telemetry.update();
            }
            */

            //for testing glyph collection; servos on either side of glyph pull or push it
            if(driver1.isButtonPressed(Button.A))
            {
                glyphMotorLeft.setPower(1.0);
                glyphMotorRight.setPower(-1.0);
                //turnTo(90);
                //rightServo.setPower(1.0);
                //leftServo.setPower(-1.0);
            }
            if(driver1.isButtonPressed(Button.B))
            {
                glyphMotorLeft.setPower(-1.0);
                glyphMotorRight.setPower(1.0);
                //turnTo(-90);
                //rightServo.setPower(-1.0);
                //leftServo.setPower(1.0);
            }

            //for robot arm test
            //servo.setPosition((gamepad1.left_stick_y + 1) / 2);

            idle();
        }
    }

}
