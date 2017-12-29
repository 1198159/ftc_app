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

        waitForStart();
        // Move jewel servo so it is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();


        // Main loop
        while(opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // Test navigation
            if (driver1.isButtonJustPressed(Button.DPAD_UP))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
                driveToPosition(0, 304.8, 0.5);
            }
            else if (driver1.isButtonJustPressed(Button.DPAD_DOWN))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
                driveToPosition(0, -508, 1.0);
            }
            else if (driver1.isButtonJustPressed(Button.DPAD_RIGHT))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
                driveToPosition(1000, 0, 0.6);
            }
            // Test turning and movement
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


            // Test glyphter
            if(driver1.isButtonJustPressed(Button.A))
            {
                //jointPosCount += 0.02;
                //jointServo.setPosition(jointPosCount);
                //motorCollectorLeft.setPower(1.0);
                //motorCollectorRight.setPower(-1.0);
                glyphMechanism.driveGlyphterToPosition(Constants.HEIGHT_1, 1.0);
            }
            if(driver1.isButtonJustPressed(Button.B))
            {
                //jointPosCount -= 0.02;
                //jointServo.setPosition(jointPosCount);

                //glyphMotorLeft.setPower(-1.0);
                //glyphMotorRight.setPower(1.0);
                glyphMechanism.driveGlyphterToPosition(Constants.HEIGHT_2, 1.0);
            }
            if(driver1.isButtonJustPressed(Button.Y))
            {
                //wristServo.setPosition(Constants.WRIST_SERVO_DEPLOYED);
                glyphMechanism.driveGlyphterToPosition(Constants.HEIGHT_3, 1.0);
            }
            if (driver1.isButtonJustPressed(Button.X))
            {
                glyphMechanism.driveGlyphterToPosition(Constants.HEIGHT_4, 1.0);
            }

            //telemetry.addData("jointPos: ", jointPosCount);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }

}
