package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.*;

/*
    Controls a drive assembly with PID-controlled speed instead of power.
*/

//TODO make use of PID Enforcement modes and remove DriveAssembly PID
public class DriveAssembly
{
    public DcMotor motor;
    public double gearRatio;   //between motor and  wheel, not gearbox
    public double wheelRadius;
    public int encoderTicks = 1120;
    public double encoderCorrectionFactor = 1.0;//correct for mismatch or reversed gearboxes
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

    public void initialize(DcMotor m, Transform2D t, double gear, double radius,double cFactor)
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
    public double getLinearEncoderDifferential()
    {
        double currentTime = System.nanoTime()/1000/1000/1000;
        double timeDiff = currentTime - lastReadTime;
        int pos  = (int)(motor.getCurrentPosition()*encoderCorrectionFactor);
        int diff = pos - lastEncoderValue;
        lastEncoderValue = pos;
        lastReadTime = currentTime;

        return pos*gearRatio*wheelRadius*2*Math.PI/encoderTicks / timeDiff;
    }

    //return a vector representing the last motion made by the drive
    public double[] getVectorEncoderDifferential()
    {
        double length = getLinearEncoderDifferential();
        double[] v = new double[]{ length * Math.cos(location.rot),
                                   length * Math.sin(location.rot)  };

        return v;
    }

    //return the angle the robot has turned due to this drive
    public double getRotationEncoderDifferential()
    {
        double length = getLinearEncoderDifferential();
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
