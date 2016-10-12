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
    public double gearRatio;//between motor and  wheel, not gearbox
    public double wheelRadius;
    public int encoderTicks = 1120;
    private int lastEncoderValue = 0;
    private double lastReadTime = 0;
    public Transform2D location;

    //construct empty
    public DriveAssembly()
    {
        this.motor = null;
        this.location = new Transform2D(0.0,0.0,0.0);
        this.gearRatio = 1.0;
        this.wheelRadius = 1.0;
        this.lastReadTime = System.nanoTime()/1000/1000/1000;
    }

    //construct with values
    public DriveAssembly(DcMotor m, Transform2D t, double gear, double radius)
    {
        this.motor = m;
        this.wheelRadius = radius;
        this.gearRatio = gear;
        this.location = t;
        this.lastReadTime = System.nanoTime()/1000/1000/1000;
    }

    public void ZeroEncoder()
    {
        //reset encoder to zero and set mode
        //since we have our own PID loop for controlling motor speed, we'll disable the built-in pid
        m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    //get the average rate of motion for this drive along its tangent in m/s
    public double getLinearEncoderDifferential()
    {
        double currentTime = System.nanoTime()/1000/1000/1000;
        double timeDiff = currentTime - lastReadTime;
        int pos  = motor.getCurrentPosition();
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
