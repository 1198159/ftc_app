package org.firstinspires.ftc.team6220;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Handles control of the gate that loads the launcher.
    Usage:
    releaseParticle() starts a concurrent operation which will raise the gate and then close it after some time.
    calling this repeatedly will hold the servo open.
*/
public class Launcher implements ConcurrentOperation
{
    //TODO update servo and motor handlers to implement ConcurrentOperation and use thise here
    private DcMotor pullBackMotor;
    private Servo gateServo;

    private String motorDevice = "";
    private String servoDevice = "servoCollectorGate";

    private boolean isLaunching = false;

    //servo gate states
    private enum GateState
    {
        OPEN,
        WAIT,
        CLOSE
    }
    private GateState gateState = GateState.OPEN;
    //keeps track of how long servo has waited before closing
    private double servoWaitTime = 0;

    private OpMode mode;
    public void releaseParticle()
    {
        gateState = GateState.OPEN; //start the state machine running
    }

    public void initialize(HardwareMap hMap)
    {
        //pullBackMotor = hMap.dcMotor.get(motorDevice);
        gateServo = hMap.servo.get(servoDevice);
        gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
    }

    //we pass the opmode so that this class can generate telemetry
    public Launcher(OpMode mode)
    {
        this.mode = mode;
    }

    @Override
    public void update(double eTime)
    {
        if (gateState == GateState.OPEN) {
            //raise gate and reset
            servoWaitTime = 0;
            gateServo.setPosition(Constants.GATE_SERVO_RETRACTED_POSITION);
            gateState = GateState.WAIT;
        }
        else if (gateState == GateState.WAIT) {
            servoWaitTime += eTime;
            //mode.telemetry.addData("open",servoWaitTime);

            if(servoWaitTime > Constants.GATE_SERVO_CLOSE_DELAY)
            {
                //mode.telemetry.addLine("close");
                servoWaitTime = 0;
                gateState = GateState.CLOSE;
                gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
            }
        }

    }
}
