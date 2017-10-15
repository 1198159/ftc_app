package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.hardware.Servo;

/*
    Handles back and forth servo motions that are common for manipulators.
*/

public class ServoToggler
{
    Servo servo;
    boolean isDeployed;
    double servoRetractedPosition;
    double servoDeployedPosition;

    public ServoToggler(Servo s, double retractedPosition, double deployedPostition)
    {
        servo = s;
        servoRetractedPosition = retractedPosition;
        servoDeployedPosition = deployedPostition;
        setStartingPosition();
    }

    public void setStartingPosition ()
    {
        retract();
    }

    public void deploy ()
    {
        servo.setPosition(servoDeployedPosition);
        isDeployed = true;
    }

    public void retract ()
    {
        servo.setPosition(servoRetractedPosition);
        isDeployed = false;
    }

    public void toggle ()
    {
        if (isDeployed)
        {
            retract();
        }
        else
        {
            deploy();
        }
    }
}
