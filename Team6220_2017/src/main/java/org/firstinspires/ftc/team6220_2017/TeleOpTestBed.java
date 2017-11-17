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
    Servo servo;
    VuforiaHelper vuforiaHelper = new VuforiaHelper();

    @Override public void runOpMode() throws InterruptedException
    {
        double jointPosAdd = 0.5;

        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        //initializeHardware();

        DcMotor glyphMotorLeft;
        DcMotor glyphMotorRight;
        Servo wristServo;
        Servo jointServo;

        //glyphMotorLeft = hardwareMap.dcMotor.get("glyphMotorLeft");
        //glyphMotorRight = hardwareMap.dcMotor.get("glyphMotorRight");
        //relicGrabberWrist = hardwareMap.servo.get("relicGrabberServo");
        wristServo = hardwareMap.servo.get("servoWrist");
        jointServo = hardwareMap.servo.get("servoJoint");

        vuforiaHelper.setupVuforia();
        // Wait until start button has been pressed
        waitForStart();
        lTime = timer.seconds();
        //relicGrabberWrist.setPosition(0.7);
        jointServo.setPosition(0.5);
        // Main loop
        while(opModeIsActive())
        {

            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            // for motor that actuates arm
            //motorArm.setPower(gamepad1.left_stick_y);

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
            if(gamepad1.a)
            {
                jointPosAdd += 0.05;
                jointServo.setPosition(jointPosAdd);

                //glyphMotorLeft.setPower(1.0);
                //glyphMotorRight.setPower(-1.0);

                //turnTo(90);
            }
            if(gamepad1.b)
            {
                jointPosAdd -= 0.05;
                jointServo.setPosition(jointPosAdd);

                //glyphMotorLeft.setPower(-1.0);
                //glyphMotorRight.setPower(1.0);

                //turnTo(-90);
            }
            if(gamepad1.x)
            {
                wristServo.setPosition(Constants.WRIST_SERVO_DEPLOYED);
            }

            telemetry.addData("jointPos: ", jointPosAdd);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }

}
