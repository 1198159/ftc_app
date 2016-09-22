package org.firstinspires.ftc.team8923;

/*
 * This class contains all objects and methods that should be accessible by all Autonomous OpModes
 * for the CapBot
 */
abstract public class MasterAutonomousCapBot extends Master
{
    // TODO: Since this method and the TeleOp method use the same math, should we combine them?
    // This allows the robot to drive in any direction and/or turn. It uses the same math as TeleOp
    public void driveMecanumAuto(double driveAngle, double drivePower, double turnPower)
    {
        double x = drivePower * Math.cos(driveAngle);
        double y = drivePower * Math.sin(driveAngle);

        // Set power for motors. Ratios are correct, but needs scaling (see below)
        double powerFL = y - x + turnPower;
        double powerFR = y + x - turnPower;
        double powerBL = y + x + turnPower;
        double powerBR = y - x - turnPower;

        // Motor powers might be set above 1, so this scales all of the motor powers to stay
        // proportional and within power range
        double scalar = Math.max(Math.abs(powerFL), Math.max(Math.abs(powerFR),
                Math.max(Math.abs(powerBL), Math.abs(powerBR))));

        // Scalar equation above doesn't account for times when partial power is requested. This
        // compensates for that
        scalar /= drivePower;

        // Don't divide by 0
        if(scalar != 0)
        {
            powerFL /= scalar;
            powerFR /= scalar;
            powerBL /= scalar;
            powerBR /= scalar;
        }

        // Set motor powers
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorBL.setPower(powerBL);
        motorBR.setPower(powerBR);
    }
}
