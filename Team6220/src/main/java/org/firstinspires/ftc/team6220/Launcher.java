package org.firstinspires.ftc.team6220;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Handles the gate that loads balls into the launcher.
    Usage:
    loadParticle() starts a concurrent operation which will raise the gate and then close it after some time.
    calling this repeatedly will hold the servo open.
*/
public class Launcher implements ConcurrentOperation
{
    //TODO update servo and motor handlers to implement ConcurrentOperation and use this here
    private DcMotor pullBackMotor;
    private Servo gateServo;

    private String motorDevice = "motorLauncher";
    private String servoDevice = "servoCollectorGate";

    private boolean isLaunching = false;

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
        WAIT,
        LAUNCH,
        IDLE
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
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gateServo = hMap.servo.get(servoDevice);
        gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
    }

    public void loadParticle()
    {
        gateState = GateState.OPEN; //start the state machine running
    }

    public void pullback()
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

        launchState = LaunchState.WAIT;
    }

    //launches particle. Calling before pullback() will make it pullback, load, and launch immediately.
    public void launchParticle() throws InterruptedException
    {
      /*  if (launchState == LaunchState.WAIT)
        {
            pullBackMotor.setTargetPosition(Constants.LAUNCHER_FIRING_POSITION);
            pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);

            //ensures that the previous motor action is completed before starting the next one
            while (pullBackMotor.isBusy() && masterOpMode.opModeIsActive())
            {
                masterOpMode.telemetry.addData("launcherEncoderVal: ", pullBackMotor.getCurrentPosition());
                masterOpMode.telemetry.update();

                masterOpMode.idle();
            }

            //resets the encoder each loop so it will display the correct values every time
            pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else */
        {
            pullback();

            //load a particle
            gateServo.setPosition(Constants.GATE_SERVO_DEPLOYED_POSITION);
            masterOpMode.pause(400);
            gateServo.setPosition(Constants.GATE_SERVO_RETRACTED_POSITION);

            //launch a particle
            pullBackMotor.setTargetPosition(Constants.LAUNCHER_FIRING_POSITION);
            pullBackMotor.setPower(Constants.LAUNCHER_SHOOT_POWER);

            //ensures that the previous motor action is completed before starting the next one
            while (pullBackMotor.isBusy() && masterOpMode.opModeIsActive())
            {
                masterOpMode.telemetry.addData("launcherEncoderVal: ", pullBackMotor.getCurrentPosition());
                masterOpMode.telemetry.update();

                masterOpMode.idle();
            }

            //resets the encoder every loop so it will display the correct values each time
            pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        launchState = LaunchState.IDLE;
    }

    public void trimForward()
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

        pullBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pullBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void trimBackward()
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

        masterOpMode.telemetry.addData("pullBackMotorEncVal: ", pullBackMotor.getCurrentPosition());

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

        masterOpMode.telemetry.update();
    }
}
