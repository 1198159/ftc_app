package org.firstinspires.ftc.team6220;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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

    private String motorDevice = "motorLauncher";
    private String servoDevice = "servoCollectorGate";

    private boolean isLaunching = false;

    //private int loopcounter=0;

    private int launchCount = 0;
    private int currentLaunchTargetStart = 0;

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
        PULLBACK,
        WAIT_WHILE_PULLBACK,
        WAIT_FOR_LAUNCH,
        LAUNCH,
        WAIT_WHILE_LAUNCHING,
        IDLE,
        QUEUED_LAUNCH
    }
    private GateState gateState = GateState.OPEN;
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
        pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gateServo = hMap.servo.get(servoDevice);
        gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
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

    @Deprecated
    public void pullbackBNotUsed()
    {
        pullBackMotor.setTargetPosition(Constants.LAUNCHER_PULLBACK_POSITION);
        pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);

        //ensures that the previous motor action is completed before starting the next one
        while (pullBackMotor.isBusy() && masterOpMode.opModeIsActive())
        {
            masterOpMode.telemetry.addData("launcherEncoderVal: ", pullBackMotor.getCurrentPosition());
            masterOpMode.telemetry.update();

            masterOpMode.idle();
        }

        launchState = LaunchState.WAIT_WHILE_PULLBACK;
    }

    //launches particle, if pulled back
    public void launchParticle()
    {
        if(launchState == LaunchState.WAIT_FOR_LAUNCH)
        {
            launchState = LaunchState.LAUNCH;
        }

    }

    public void trimForward() throws InterruptedException
    {
        pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pullBackMotor.setTargetPosition(Constants.LAUNCHER_TRIM_INTERVAL);
        pullBackMotor.setPower(Constants.LAUNCHER_TRIM_POWER);

        while(pullBackMotor.isBusy() && masterOpMode.opModeIsActive())
        {
            masterOpMode.telemetry.addData("launcherEncoderVal: ", pullBackMotor.getCurrentPosition());
            masterOpMode.telemetry.update();

            masterOpMode.idle();
        }

        masterOpMode.pause(100);
        pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void trimBackward() throws InterruptedException
    {
        pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pullBackMotor.setTargetPosition(-Constants.LAUNCHER_TRIM_INTERVAL);
        pullBackMotor.setPower(-Constants.LAUNCHER_TRIM_POWER);

        while(pullBackMotor.isBusy() && masterOpMode.opModeIsActive())
        {
            masterOpMode.telemetry.addData("launcherEncoderVal: ", pullBackMotor.getCurrentPosition());
            masterOpMode.telemetry.update();

            masterOpMode.idle();
        }
        masterOpMode.pause(100);
        pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

        int pullMotorPosition = pullBackMotor.getCurrentPosition();
        masterOpMode.telemetry.addData("pullBackMotorEncVal: ", pullMotorPosition);


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
        if(launchState == LaunchState.PULLBACK)
        {
            pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);
            setMotorTargetOffset(pullBackMotor, Constants.LAUNCHER_PULLBACK_POSITION);
            currentLaunchTargetStart = pullMotorPosition;
            launchState = LaunchState.WAIT_WHILE_PULLBACK;
        }

        if (launchState == LaunchState.WAIT_WHILE_PULLBACK)
        {
            //masterOpMode.telemetry.addLine("Waiting: Pulling back." + loopcounter++);
            if(isWithinTolerance(pullMotorPosition, currentLaunchTargetStart + Constants.LAUNCHER_PULLBACK_POSITION, 20))
            {
                //now, wait for a launch
                launchState = LaunchState.WAIT_FOR_LAUNCH;
                //masterOpMode.telemetry.addLine("Waiting to launch." + loopcounter++);
            }
        }
        //if launch requested and already pulled back
        if(launchState == LaunchState.LAUNCH) {
            pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);
            setMotorTargetOffset(pullBackMotor,Constants.LAUNCHER_FIRING_POSITION);
            currentLaunchTargetStart = pullMotorPosition;
            launchCount++;
            launchState = LaunchState.WAIT_WHILE_LAUNCHING;
        }

        if (launchState == LaunchState.WAIT_WHILE_LAUNCHING)
        {
            //masterOpMode.telemetry.addLine("Waiting: Launching." + loopcounter++);
            if(isWithinTolerance(pullMotorPosition,currentLaunchTargetStart+Constants.LAUNCHER_FIRING_POSITION,20))
            {
                //launch is finished. do nothing
                masterOpMode.telemetry.addLine("Launcher idling.");
                launchState = LaunchState.IDLE;
                //pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //pullBackMotor.setTargetPosition(0);
            }
        }

    }

}
