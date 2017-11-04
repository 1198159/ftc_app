package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input
*/
abstract public class MasterTeleOp extends MasterOpMode
{
    // polynomial for adjusting input from joysticks to allow for ease of use
    //                                         y = 0.0 + (3/10)x + 0.0 + (7/10)x^3
    Polynomial stickCurve = new Polynomial(new double[]{ 0.0, 0.3, 0.0, 0.7 });

    //takes driver 1 stick input and uses it to give power and direction inputs to the drive
    void driveMecanumWithJoysticks()
    {
        // factor changing magnitude of vertical and horizontal movement
        double tFactor;
        // factor changing magnitude of rotational movement
        double rFactor;

        /*
         slow mode functionality
         note: factors are different for translation and rotation
        */
        if (driver1.isButtonPressed(Button.RIGHT_BUMPER))
        {
            tFactor = Constants.SLOW_MODE_T_FACTOR;
            rFactor = Constants.SLOW_MODE_R_FACTOR;
        }
        else
        {
            tFactor = Constants.T_FACTOR;
            rFactor = Constants.R_FACTOR;
        }

        /*
         adjust stick magnitude, since stick magnitudes are not exactly
         1 (they have been observed to be as large as 1.1)
        */
        double adjustedStickMagnitude = Range.clip(driver1.getRightStickMagnitude(), -1.0, 1.0);

        // stick inputs must be changed from x and y to angle and drive power
        double angle = driver1.getRightStickAngle();
        double power = tFactor * stickCurve.getOuput(adjustedStickMagnitude);
        double rotationPower = rFactor * stickCurve.getOuput(gamepad1.left_stick_x);

        driveMecanum(angle, power, rotationPower);
    }

    public void driveArm()
    {
        // stores input from sticks to properly power turnTableServo
        double turnTablePosCount = 0.5;

        if (driver2.isButtonJustPressed(Button.LEFT_BUMPER))
        {
            hingeServoToggler.toggle();
        }
        else if(driver2.isButtonJustPressed(Button.RIGHT_BUMPER))    // for grabbing glyphs
        {
            grabberServoToggler.toggle();
        }
        else if (driver2.isButtonJustPressed(Button.Y))              // for grabbing relic
        {
            grabberServoToggler.deployToAlternatePosition(Constants.GRABBER_SERVO_RELIC);
        }
        else if (Math.abs(gamepad2.right_stick_y) > Constants.MINIMUM_JOYSTICK_POWER_ARM)
        {
            // adjust power inputs for the arm motor
            double adjustedStickPower = 0.75 * Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
            double armPower = stickCurve.getOuput(adjustedStickPower);
            motorArm.setPower(armPower);

            telemetry.addData("armPower: ", armPower);
        }
        else if (Math.abs(gamepad2.left_stick_x) >= Constants.MINIMUM_JOYSTICK_POWER_ARM)
        {
            turnTablePosCount += Constants.TURN_TABLE_POS_COUNT_STEP_SIZE * stickCurve.getOuput(gamepad2.left_stick_x);

            // ensures turnTablePosCount does not go beyond acceptable servo position range
            if (turnTablePosCount > 1.0)
                turnTablePosCount = 1.0;
            else if (turnTablePosCount < 0.0)
                turnTablePosCount = 0.0;

            turnTableServo.setPosition(turnTablePosCount);

            telemetry.addData("turnTablePosCount: ", turnTablePosCount);
        }
        else if (Math.abs(gamepad2.left_stick_x) < Constants.MINIMUM_JOYSTICK_POWER_ARM)  //todo change turnTableServo to CR to make code simpler; this is somewhat sloppy
        {
            turnTableServo.setPosition(turnTableServo.getPosition());
        }

        telemetry.update();
    }
}
