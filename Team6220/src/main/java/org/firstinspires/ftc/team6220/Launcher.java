package org.firstinspires.ftc.team6220;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements ConcurrentOperation
{
    //TODO update servo and motor handlers to implement ConcurrentOperation and use thise here
    private DcMotor pullBackMotor;
    private Servo gateServo;
    private int b = 0;

    private String motorDevice = "";
    private String servoDevice = "servoCollectorGate";

    private boolean isLaunching = false;

    //variables that define states of a state machine that releases a ball into the launcher
    //the states are isReleasing, isWaiting (and then stopped)
    private boolean isReleasing = false;
    private boolean isWaiting = false;
    private double servoWaitTime = 0;

    private OpMode mode;
    public void releaseParticle()
    {
        isReleasing = true; //start the state machine running
    }

    public void initialize(HardwareMap hMap)
    {
        //pullBackMotor = hMap.dcMotor.get(motorDevice);
        gateServo = hMap.servo.get(servoDevice);
        gateServo.setPosition(Constants.COLLECTOR_GATE_SERVO_DEPLOYED_POSITION);
    }

    public Launcher(OpMode mode)
    {
        this.mode = mode;
    }

    @Override
    public void update(double eTime)
    {
        b++;
        mode.telemetry.addData("did callback for launcher",b);
        if (isReleasing) {
            gateServo.setPosition(Constants.COLLECTOR_GATE_SERVO_RETRACTED_POSITION);
            isReleasing = false;
            isWaiting = true;
        }
        if (isWaiting) {
            servoWaitTime += eTime;
            mode.telemetry.addData("open",servoWaitTime);
            if(servoWaitTime > 0.4)
            {
                mode.telemetry.addLine("close");
                servoWaitTime = 0;
                isWaiting = false;
                isReleasing=false;
                gateServo.setPosition(Constants.COLLECTOR_GATE_SERVO_DEPLOYED_POSITION);
            }
        }

    }
}
