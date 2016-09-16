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
    Transform2D transform;

    //construct empty
    public DriveAssembly()
    {
        this.motor = null;
        this.transform = new Transform2D(0.0,0.0,0.0);
        this.gearRatio = 1.0;
    }

    //construct with values
    public DriveAssembly(DcMotor m, Transform2D t, double gear)
    {
        this.motor = m;
        this.transform = t;
        this.gearRatio = gear;
    }

    //get the output of the motor
    public Vector3D getDrivingVector()
    {
        return Vector3D.yAxis.matrixMultiplied(transform.getMatrix());
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
