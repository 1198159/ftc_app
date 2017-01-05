package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.*;

/*
    Controls a drive assembly with PID-controlled speed instead of power.
*/

public class DriveAssembly
{
    public DcMotor motor;
    public double gearRatio;   //between motor and  wheel, not gearbox
    public double wheelRadius;
    //this value is only for andymark 20 motors; should be changed if used for other motors
    public static final int encoderTicks = 560;
    public double encoderCorrectionFactor = 1.0;//correct for mismatch or reversed gearboxes  //CodeReview: make this final so it's a constant
    private int lastEncoderValue = 0;
    private double lastReadTime = 0;
    public Transform2D location;

    //construct an empty DriveAssembly
    public DriveAssembly()
    {
        this.initialize(null, new Transform2D(0.0,0.0,0.0), 1.0, 1.0, 1.0);
    }

    //construct a DriveAssembly with values added
    public DriveAssembly(DcMotor m, Transform2D t, double gear, double radius,double cFactor)
    {
        this.initialize(m, t, gear, radius,cFactor);
    }

    public void initialize(DcMotor m, Transform2D t, double gear, double radius, double cFactor)
    {
        this.motor = m;
        this.wheelRadius = radius;
        this.gearRatio = gear;
        this.location = t;
        this.lastReadTime = System.nanoTime()/1000/1000/1000;
        this.encoderCorrectionFactor = cFactor;
        zeroEncoder();
    }

    public void zeroEncoder()
    {
        //reset encoder to zero and set mode
        //since we have our own PID loop for controlling motor speed, we'll disable the built-in pid
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //get the average rate of motion for this drive along its tangent in m/s
    public double getEncoderLinearDerivative(double eTime)
    {
        int pos  = (int)(motor.getCurrentPosition() * encoderCorrectionFactor);
        int diff = pos - lastEncoderValue;
        lastEncoderValue = pos;

        return pos * gearRatio * wheelRadius * 2 * Math.PI / encoderTicks / eTime;
    }

    //return a vector representing the last motion made by the drive
    public double[] getEncoderVectorDerivative(double eTime)
    {
        double length = getEncoderLinearDerivative(eTime);
        double[] v = new double[]{ length * Math.cos(location.rot * Constants.DEGREE_TO_RADIAN),
                                   length * Math.sin(location.rot * Constants.DEGREE_TO_RADIAN) };

        return v;
    }

    //return the angle the robot has turned due to this drive
    public double getEncoderRadialDifferential(double eTime)
    {
        double length = getEncoderLinearDerivative(eTime);
        double radius = location.getPositionVector().magnitude();
        return length/radius/Math.PI*180;
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
