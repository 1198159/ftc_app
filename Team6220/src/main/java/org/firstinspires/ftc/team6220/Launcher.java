package org.firstinspires.ftc.team6220;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements ConcurrentOperation
{
    //TODO update servo and motor handlers to implement ConcurrentOperation and use thise here
    private DcMotor pullBackMotor;
    private Servo gateServo;

    private String motorDevice = "";
    private String servoDevice = "servoCollectorGate";

    private boolean isLaunching = false;
    private boolean isReleasing = false;
    private double servoWaitTime = 0;

    public void releaseParticle()
    {
        isReleasing = true;
    }

    public void initialize(HardwareMap hMap)
    {
        //pullBackMotor = hMap.dcMotor.get(motorDevice);
        gateServo = hMap.servo.get(servoDevice);
        gateServo.setPosition(Constants.COLLECTOR_GATE_SERVO_DEPLOYED_POSITION);
    }

    @Override
    public void update(double eTime)
    {
        if (isReleasing)
        {
            gateServo.setPosition(Constants.COLLECTOR_GATE_SERVO_RETRACTED_POSITION);
            servoWaitTime += eTime;
            if(servoWaitTime > 1.0)
            {
                isReleasing = false;
            }
        }
        else
        {
            gateServo.setPosition(Constants.COLLECTOR_GATE_SERVO_DEPLOYED_POSITION);
        }

    }
}
