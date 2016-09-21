package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.*;

/*
    Controls a drive assembly with PID controlled speed instead of power.
*/

//TODO make use of PID Enforcement modes and remove DriveAssembly PID
public class DriveAssembly
{
    public DcMotor motor;
    public double gearRatio;
    public double wheelRadius;
    public int encoderTicks = 1024;
    public Transform2D location;

    //construct empty
    public DriveAssembly()
    {
        this.motor = null;
        this.location = new Transform2D(0.0,0.0,0.0);
        this.gearRatio = 1.0;
        this.wheelRadius = 1.0;
    }

    //construct with values
    public DriveAssembly(DcMotor m, Transform2D t, double gear, double radius)
    {
        this.motor = m;
        this.wheelRadius = radius;
        this.location = t;
        this.gearRatio = gear;
    }

    //get the output of the motor
    public VectorF getDrivingVector()
    {
        float x = (float)(Math.cos(location.rot)*this.motor.getPower());
        float y = (float)(Math.sin(location.rot)*this.motor.getPower());
        return new VectorF(x,y,0);
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
