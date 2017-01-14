package org.firstinspires.ftc.team6220;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import static org.firstinspires.ftc.team6220.Utility.*;

/*
    Handles the gate that loads balls into the launcher.
    Usage:
    loadParticle() starts a concurrent operation which will raise the gate and then close it after some time.
    calling this repeatedly will hold the servo open.
*/
public class Launcher implements ConcurrentOperation
{
    //TODO update servo and motor handlers to implement ConcurrentOperation and use this here
    public DcMotor pullBackMotor;
    private Servo gateServo;
    private TouchSensor button;

    private String motorDevice = "motorLauncher";
    private String servoDevice = "servoCollectorGate";
    private String buttonDevice = "button";

    private boolean isLaunching = false;

    private int launchCount = 0;
    private int currentLaunchTargetStart = 0;
    private double lst = 0.0;

    //servo gate states
    private enum GateState
    {
        OPEN,
        WAIT,
        CLOSE
    }

    //launching states
    private enum LaunchState
    {
        IDLE,
        PULLBACK,
        LAUNCH
    }
    private GateState gateState = GateState.CLOSE;
    private LaunchState launchState = LaunchState.IDLE;

    //keeps track of how long servo has waited before closing
    private double servoWaitTime = 0;
    private double motorWaitTime = 0;

    private MasterOpMode masterOpMode;

    //used to initialize objects that are part of the launcher
    public void initialize(HardwareMap hMap)
    {
        pullBackMotor = hMap.dcMotor.get(motorDevice);
        pullBackMotor.setPower(0);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        gateServo = hMap.servo.get(servoDevice);
        gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
        button = hMap.touchSensor.get(buttonDevice);
    }

    //load particle, if launcher is cabable
    public void loadParticle()
    {
        //if(true)//launchState == LaunchState.WAIT)
        {
            gateState = GateState.OPEN; //start the state machine running
        }
    }

    //cock the launcher, if the launcher is not either launching or already cocked
    public void pullback()
    {
        launchState = LaunchState.PULLBACK;
    }


    //launches particle, if pulled back
    public void launchParticle()
    {
        launchState = LaunchState.LAUNCH;
    }

    //we pass the opmode so that this class can generate telemetry
    public Launcher(MasterOpMode mode)
    {
        this.masterOpMode = mode;
    }

    //sets the gate servo to the correct position at the end of each loop
    @Override
    public void update(double eTime)
    {
        //loading state machine
        if (gateState == GateState.OPEN)
        {
            //raise gate and reset
            servoWaitTime = 0;
            gateServo.setPosition(Constants.GATE_SERVO_RETRACTED_POSITION);
            gateState = GateState.WAIT;
        }
        else if (gateState == GateState.WAIT)
        {
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

        //launching state machine

        //if pullback requested
        if (launchState == LaunchState.PULLBACK)
        {
            //if NOT pressed
            if(!button.isPressed())
            {
                pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);
            }
            //if pulled back and in loading position
            else
            {
                pullBackMotor.setPower(0.0);
            }
        }

        //if launch requested and already pulled back
        if (launchState == LaunchState.LAUNCH)
        {
            motorWaitTime += eTime;

            if(button.isPressed())
            {
                pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);
            }
            //and if motor has moved for long enough anyway after clearing button
            else if (motorWaitTime > 0.4)
            {
                launchState = LaunchState.IDLE;
            }
        }

        if (launchState == LaunchState.IDLE)
        {
            motorWaitTime = 0.0;

            pullBackMotor.setPower(0.0);
        }

    }

}
