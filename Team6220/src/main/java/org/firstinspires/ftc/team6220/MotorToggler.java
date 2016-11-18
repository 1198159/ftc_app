package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Colew on 11/8/2016.
 */

public class MotorToggler
{
    DcMotor motor;
    boolean isPowered;
    double onPower;

    public MotorToggler(DcMotor s, double power)
    {
        motor = s;
        onPower = power;
        //CodeReview: initialize isPowered too. (Even though it does default to false, it's better to be explicit)
    }

    public void turnOn ()
    {
        motor.setPower(onPower);
        isPowered = true;
    }

    public void turnOff ()
    {
        motor.setPower(0.0);
        isPowered = false;
    }

    public void toggleMotor ()
    {
        if (isPowered)
        {
            turnOff();
        }
        else
        {
            turnOn();
        }
    }
}
