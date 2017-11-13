package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Mridula on 11/4/2017.
 */

abstract public class ArmMechanism extends MasterOpMode
{
    double lastTurntableValue = 0.0;
    // Method for running arm system on robot
    public void driveArm()
    {
        if(!isArmAttached)
        {
            return;
        }
        if (driver2.isButtonJustPressed(Button.LEFT_BUMPER))
        {
            hingeServoToggler.toggle();
        }
        else if(driver2.isButtonJustPressed(Button.RIGHT_BUMPER))    // For grabbing glyphs
        {
            grabberServoToggler.toggle();
        }
        else if (driver2.isButtonJustPressed(Button.Y))              // For grabbing relic
        {
            grabberServoToggler.deployToAlternatePosition(Constants.GRABBER_SERVO_RELIC);
        }
        else if(driver2.isButtonJustPressed(Button.B))
        {
            grabberServoToggler.servoDeployedPosition += 0.1;

            if (grabberServoToggler.servoDeployedPosition > 1.0)
                grabberServoToggler.servoDeployedPosition = 1.0;

            grabberServoToggler.deploy();
        }
        else if(driver2.isButtonJustPressed(Button.X))
        {
            grabberServoToggler.servoDeployedPosition -= 0.1;

            if (grabberServoToggler.servoDeployedPosition < 0.0)
                grabberServoToggler.servoDeployedPosition = 0.0;

            grabberServoToggler.deploy();
        }
        else if (Math.abs(gamepad2.right_stick_y) > Constants.MINIMUM_JOYSTICK_POWER)
        {
            // Adjust power inputs for the arm motor
            double adjustedStickPower = Constants.ARM_POWER_CONSTANT * Range.clip(gamepad2.right_stick_y, -1.0, 1.0);
            double armPower = stickCurve.getOuput(adjustedStickPower);
            motorArm.setPower(armPower);

            telemetry.addData("armPower: ", armPower);
        }

        // Move turntable------------------
        double pow = gamepad2.left_stick_x;

        // Ensure that tiny joystick twitches do not make the servo drift
        if (Math.abs(pow) < Constants.MINIMUM_JOYSTICK_POWER)
            pow = 0.0;

        double adjustedPow = stickCurve.getOuput(pow);

        // Only send command to servo if the value has changed
        if (lastTurntableValue != adjustedPow)
        {
            lastTurntableValue = adjustedPow;
            turnTableServo.setPower(lastTurntableValue);
        }
        //---------------------------------

        telemetry.update();
    }
}
