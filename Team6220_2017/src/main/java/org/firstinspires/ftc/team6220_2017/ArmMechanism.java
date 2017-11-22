package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Mridula on 11/4/2017.
 */

public class ArmMechanism
{
    MasterOpMode masterOpMode;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public ArmMechanism (MasterOpMode mode)
    {
        this.masterOpMode = mode;
    }

    // Method for running entire arm system on robot
    public void driveArm()
    {
        if(!masterOpMode.isArmAttached)
        {
            masterOpMode.telemetry.addLine("Arm is not attached!");
            masterOpMode.telemetry.update();
            return;
        }

        // todo Add code for wrist and joint
        // Run arm motor, wrist, and joint
        if (masterOpMode.gamepad2.right_stick_y > Constants.MINIMUM_JOYSTICK_POWER)
        {
            // Adjust power inputs for the arm motor
            double adjustedStickPower = Constants.ARM_POWER_CONSTANT * Range.clip(masterOpMode.gamepad2.right_stick_y, -1.0, 1.0);
            double armPower = masterOpMode.stickCurve.getOuput(adjustedStickPower);
            masterOpMode.motorArm.setPower(armPower);

            masterOpMode.telemetry.addData("armPower: ", armPower);
        }

        masterOpMode.telemetry.update();
    }
}
