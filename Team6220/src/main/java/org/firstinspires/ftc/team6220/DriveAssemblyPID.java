package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
    Controls a drive assembly with PID controlled speed instead of power.
*/

//TODO make use of PID Enforcement modes and remove DriveAssembly PID
public class DriveAssemblyPID extends DriveAssembly {

    private PIDFilter velocityFilter;
    private double positions[] = {0.0,0.0};
    //time before
    private double lastTime = 0.0;

    public DriveAssemblyPID(DcMotor m, Transform2D t, double gear, double p, double i, double d)
    {
        this.motor = m;
        this.location = t;
        this.gearRatio = gear;
        this.velocityFilter = new PIDFilter(p,i,d);
        //record time in seconds
        //we are not using millis() due to inconsistencies in short, sub-second intervals
        lastTime = System.nanoTime()/1000/1000/1000;
    }

    //set target speed in rads/sec
    public void update(float targetSpeed){
        //elapsed time since last loop, in seconds
        double lastInterval = System.nanoTime()/1000/1000/1000 - lastTime;
        lastTime = System.nanoTime()/1000/1000/1000;

        //calculate current speed with encoder
        positions[1] = positions[0];
        positions[0] = motor.getCurrentPosition()*Math.PI/encoderTicks/gearRatio;
        double vel = (positions[0]-positions[1])/lastInterval;

        velocityFilter.roll(targetSpeed-vel);
        setPower(velocityFilter.getFilteredValue());
    }


}
