package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Utility
{
    public static boolean isWithinTolerance(double value, double target, double tolerance)
    {
        return (Math.abs(value-target) < tolerance);
    }
    public static double[] normalizedComponentsFromAngle(double angle)
    {
        return new double[]{Math.cos(angle),Math.sin(angle)};
    }
    public static void setMotorTargetOffset(DcMotor motor, int offset)
    {
        motor.setTargetPosition(motor.getCurrentPosition()+offset);
    }
}
