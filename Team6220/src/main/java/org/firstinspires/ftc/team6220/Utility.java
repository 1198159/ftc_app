package org.firstinspires.ftc.team6220;

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
}
