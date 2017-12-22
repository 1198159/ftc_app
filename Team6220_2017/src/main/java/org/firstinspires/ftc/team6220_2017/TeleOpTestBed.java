package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * A test program used to try out any new ideas.
 */
@TeleOp(name="TeleOp TestBed", group = "6220")
//@Disabled
public class TeleOpTestBed extends MasterAutonomous
{
    VuforiaHelper vuforiaHelper = new VuforiaHelper();

    @Override public void runOpMode() throws InterruptedException
    {
        double jointPosCount = 0.5;
        int glyphPosCount = 0;

        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Don't want to call nonexistent hardware devices in test program
        isArmAttached = false;
        isDriveTrainAttached = true;
        isGlyphMechAttached = true;
        initializeRobot();

        DcMotor motorCollectorLeft;
        DcMotor motorCollectorRight;

        //motorCollectorLeft = hardwareMap.dcMotor.get("motorCollectorLeft");
        //motorCollectorRight = hardwareMap.dcMotor.get("motorCollectorRight");
        //wristServo = hardwareMap.servo.get("wristServo");
        //jointServo = hardwareMap.servo.get("jointServo");

        waitForStart();

        //jointServo.setPosition(0.5);

        // Main loop
        while(opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            // for motor that actuates arm
            //motorArm.setPower(gamepad1.left_stick_y);

            // Test jewel servos and encoders

            if (driver1.isButtonJustPressed(Button.DPAD_UP))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
                driveToPosition(0, 1000, 0.3);
            }
            else if (driver1.isButtonJustPressed(Button.DPAD_DOWN))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
                driveToPosition(0, -1000, 0.3);
            }
            else if (driver1.isButtonJustPressed(Button.LEFT_STICK_PRESS))
            {
                turnTo(90);
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
            }
            else  if (driver1.isButtonJustPressed(Button.RIGHT_STICK_PRESS))
            {
                turnTo(-90);
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
            }
            else if (driver1.isButtonJustPressed(Button.LEFT_BUMPER))
            {
                moveRobot(90, 0.3, 1.2);
                //verticalJewelServo.setPosition(Constants.VERTICAL_JEWEL_SERVO_DEPLOYED);
            }
            else if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER))
            {
                moveRobot(-90, 0.3, 1.2);
            }



            // Test glyph collection; servos on either side of glyph pull or push it
            if(driver1.isButtonJustPressed(Button.A))
            {
                //jointPosCount += 0.02;
                //jointServo.setPosition(jointPosCount);
                //motorCollectorLeft.setPower(1.0);
                //motorCollectorRight.setPower(-1.0);
                motorGlyphter.setTargetPosition(-2000);

                motorGlyphter.setPower(0.5);
                //turnTo(90);
            }
            if(driver1.isButtonJustPressed(Button.B))
            {
                //jointPosCount -= 0.02;
                //jointServo.setPosition(jointPosCount);

                //glyphMotorLeft.setPower(-1.0);
                //glyphMotorRight.setPower(1.0);
                motorGlyphter.setTargetPosition(-2000);
                motorGlyphter.setPower(-0.5);
                //turnTo(-90);
            }
            if(driver1.isButtonJustPressed(Button.X))
            {
                motorGlyphter.setTargetPosition(2000);
                motorGlyphter.setPower(-0.5);
                //wristServo.setPosition(Constants.WRIST_SERVO_DEPLOYED);
            }
            if (driver1.isButtonJustPressed(Button.Y))
            {
                motorGlyphter.setTargetPosition(2000);
                motorGlyphter.setPower(0.5);
                //verticalJewelServoToggler.toggle();
                //pauseWhileUpdating(0.5);
            }

            telemetry.addData("EncoderCount", motorGlyphter.getCurrentPosition()); // read encoder counts to update the count displayed
            //telemetry.update(); // display the encoder count to the driver station phone screen

            //telemetry.addData("jointPos: ", jointPosCount);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }

}
