package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
    Controls a drive assembly with PID controlled speed instead of power.
*/

public class DriveAssembly
{
    public DcMotor motor;
    public double gearRatio;
    public double power = 0.0;
    public int encoderTicks = 1024;
    Mat3x3 transform;

    //construct empty
    public DriveAssembly()
    {
        this.motor = null;
        this.transform = new Mat3x3(Vector2D.zeroVector,0.0);
        this.gearRatio = 1.0;
    }

    //construct with values
    public DriveAssembly(DcMotor m, Mat3x3 t, double gear)
    {
        this.motor = m;
        this.transform = t;
        this.gearRatio = gear;
    }

    //get the output of the motor
    public Vector2D getDrivingVector()
    {
        return (Vector2D) Vector2D.yAxis.matrixMultiplied(transform);
    }

    public void setPower(double p)
    {
        this.motor.setPower(p);
    }

    public void stop()
    {
        this.motor.setPower(0.0);
    }

}
