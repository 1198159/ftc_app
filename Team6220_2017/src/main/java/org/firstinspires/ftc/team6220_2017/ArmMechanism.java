package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

/**
 *  Encapsulates driver input for operating relic arm
 */

public class ArmMechanism
{
    MasterOpMode op;

    // todo Adjust
    double wristServoCount = 0.5;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public ArmMechanism (MasterOpMode mode)
    {
        this.op = mode;
    }

    // Method for running entire arm system on robot
    public void driveArm()
    {
        // If the arm is not attached, don't try to drive it
        if (!op.isArmAttached)
        {
            op.telemetry.addLine("Arm is not attached!");
            op.telemetry.update();
            return;
        }


        // todo Add code for wrist and joint
        // Run arm using arm motor
        if (op.driver2.getLeftStickMagnitude() > Constants.MINIMUM_JOYSTICK_POWER)
        {
            // Adjust power inputs for the arm motor
            double armPower = Constants.ARM_POWER_CONSTANT * op.stickCurve.getOuput(op.gamepad2.right_stick_y);
            op.motorArm.setPower(armPower);

            op.telemetry.addData("armPower: ", armPower);
        }

        // Run wrist using wrist servo
        if (op.driver2.getLeftTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
        {
            // todo Adjust
            wristServoCount -= 0.008;
            op.wristServo.setPosition(wristServoCount);
        }
        else if (op.driver2.getRightTriggerValue() >= Constants.MINIMUM_TRIGGER_VALUE)
        {
            // todo Adjust
            wristServoCount += 0.008;
            op.wristServo.setPosition(wristServoCount);
        }

        op.telemetry.addData("wristServoCount: ", wristServoCount);
        op.telemetry.update();
    }
}
