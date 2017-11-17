package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

/**
 * Created by Mridula on 11/4/2017.
 */

abstract public class ArmMechanism extends MasterOpMode
{
    // Method for running entire arm system on robot
    public void driveArm()
    {
        if(!isArmAttached)
        {
            return;
        }
        if (driver2.isButtonJustPressed(Button.LEFT_BUMPER))
        {
            wristServoToggler.toggle();
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

        telemetry.update();
    }
}
