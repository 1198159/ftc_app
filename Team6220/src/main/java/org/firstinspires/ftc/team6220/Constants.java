package org.firstinspires.ftc.team6220;

/**
     Used to store important constants for easy access in our programs.
 */

public class Constants
{
    public static double degToRadConversionFactor = Math.PI / 180;

    //used for PID loops involving robation and turning power adjustments
    public static double turningPowerFactor = 1 / 350;

    public static double minimumTurningPower = 0.08;
    public static double minimumAngleDiff = 3.0;
    public static double xTolerance = .010;
    public static double yTolerance = .010;
    public static double wTolerance = 3.0;
}
