package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
    Controls a drive assembly with PID controlled speed instead of power.
*/

public class DriveAssemblyPID extends DriveAssembly {

    private PIDFilter velocityFilter;
    private double positions[] = {0.0,0.0};

    public DriveAssemblyPID(DcMotor m, Transform2D t, double gear, double p, double i, double d)
    {
        this.motor = m;
        this.position = t;
        this.gearRatio = gear;
        this.velocityFilter = new PIDFilter(p,i,d);
    }

    //set target speed in rads/sec
    public void update(float targetSpeed){
        //calculate current speed with encoder
        positions[1] = positions[0];
        positions[0] = motor.getCurrentPosition()*Math.PI/1024/gearRatio;
        double vel = positions[0]-positions[1];

        velocityFilter.roll(targetSpeed-vel);
        setPower(velocityFilter.getFilteredValue());
    }


}
