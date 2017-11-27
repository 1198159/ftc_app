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

        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        // Don't want to call nonexistent hardware devices in test program
        isArmAttached = false;
        isDriveTrainAttached = false;
        isGlyphMechAttached = false;
        initializeRobot();

        DcMotor motorCollectorLeft;
        DcMotor motorCollectorRight;

        //motorCollectorLeft = hardwareMap.dcMotor.get("motorCollectorLeft");
        //motorCollectorRight = hardwareMap.dcMotor.get("motorCollectorRight");
        wristServo = hardwareMap.servo.get("wristServo");
        jointServo = hardwareMap.servo.get("jointServo");

        waitForStart();

        jointServo.setPosition(0.5);

        // Main loop
        while(opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            // for motor that actuates arm
            //motorArm.setPower(gamepad1.left_stick_y);

            // Test jewel servos
            /*if (driver1.isButtonJustPressed(Button.Y))
            {
                verticalJewelServoToggler.toggle();
                pauseWhileUpdating(0.5);
            }
            else if (driver1.isButtonJustPressed(Button.DPAD_UP))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_LEFT);
                driveToPosition(0, 1000, 0.5);
            }
            else if (driver1.isButtonJustPressed(Button.DPAD_RIGHT))
            {
                //lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_RIGHT);
                driveToPosition(1000, 0, 0.5);
            }
            else if (driver1.isButtonJustPressed(Button.RIGHT_BUMPER))
            {
                turnTo(-90);
            }
            else  if (driver1.isButtonJustPressed(Button.LEFT_BUMPER))
            {
                turnTo(90);
            }*/


            // Test glyph collection; servos on either side of glyph pull or push it
            if(driver1.isButtonJustPressed(Button.A))
            {
                jointPosCount += 0.02;
                jointServo.setPosition(jointPosCount);

                //glyphMotorLeft.setPower(1.0);
                //glyphMotorRight.setPower(-1.0);

                //turnTo(90);
            }
            if(driver1.isButtonJustPressed(Button.B))
            {
                jointPosCount -= 0.02;
                jointServo.setPosition(jointPosCount);

                //glyphMotorLeft.setPower(-1.0);
                //glyphMotorRight.setPower(1.0);

                //turnTo(-90);
            }
            if(driver1.isButtonJustPressed(Button.X))
            {
                wristServo.setPosition(Constants.WRIST_SERVO_DEPLOYED);
            }


            telemetry.addData("jointPos: ", jointPosCount);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }

}
