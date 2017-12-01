package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.util.Range;

/**
 *  Encapsulates driver input for moving relic arm
 */

public class ArmMechanism
{
    MasterOpMode op;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public ArmMechanism (MasterOpMode mode)
    {
        this.op = mode;
    }

    // Method for running entire arm system on robot
    public void driveArm()
    {
        if(!op.isArmAttached)
        {
            op.telemetry.addLine("Arm is not attached!");
            op.telemetry.update();
            return;
        }

        // todo Add code for wrist and joint
        // Run arm motor, wrist, and joint
        if (op.gamepad2.right_stick_y > Constants.MINIMUM_JOYSTICK_POWER)
        {
            // Adjust power inputs for the arm motor
            double adjustedStickPower = Constants.ARM_POWER_CONSTANT * Range.clip(op.gamepad2.right_stick_y, -1.0, 1.0);
            double armPower = op.stickCurve.getOuput(adjustedStickPower);
            op.motorArm.setPower(armPower);

            op.telemetry.addData("armPower: ", armPower);
        }

        op.telemetry.update();
    }
}
