package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CollectorMechanism implements ConcurrentOperation
{
    DriverInput driverInput;
    CRServo collector;
    ConcurrentDigitalDevice channel;
    ElapsedTime timer;

    // Collector operation booleans
    boolean collectorEncoderState = false;
    boolean collectorSlowMode = false;
    boolean isCollectorStopping = false;
    boolean isCollecting = false;
    boolean isCollectingIn = false;
    // Allows us to break out of collector encoder loop if necessary.
    ElapsedTime collectorLoopTimer = new ElapsedTime();

    double collectorPowerIn = Constants.MOTOR_COLLECTOR_IN;
    double collectorPowerOut = Constants.MOTOR_COLLECTOR_OUT;

    public CollectorMechanism(DriverInput driver2, CRServo motorCollector, ConcurrentDigitalDevice collectorChannel, ElapsedTime collectorLoopTimer)
    {
        collector = motorCollector;
        channel = collectorChannel;
        timer = collectorLoopTimer;
        driverInput = driver2;
    }

    // Not in use.
    public void initialize(HardwareMap hMap){}


    // Uses driver 2 input to drive arm and collector motors.
    // Call at end of loop.
    public void update(double etime)
    {
        if (driverInput.isButtonJustPressed(Button.RIGHT_BUMPER) && !collectorSlowMode)
        {
            collectorPowerIn = Constants.MOTOR_COLLECTOR_SLOW_IN;
            collectorPowerOut = Constants.MOTOR_COLLECTOR_SLOW_OUT;
            collectorSlowMode = true;
        }
        else if (driverInput.isButtonJustPressed(Button.RIGHT_BUMPER) && collectorSlowMode)
        {
            collectorPowerIn = Constants.MOTOR_COLLECTOR_IN;
            collectorPowerOut = Constants.MOTOR_COLLECTOR_OUT;
            collectorSlowMode = false;
        }

        // Collect and eject minerals.  Buttons have to be held to power collector.
        if (driverInput.isButtonPressed(Button.DPAD_DOWN))
        {
            collector.setPower(collectorPowerIn);
            isCollectingIn = true;
        }
        else if (driverInput.isButtonPressed(Button.DPAD_UP))
        {
            collector.setPower(collectorPowerOut);
            isCollectingIn = false;
        }
        else
        {
            // Run motor in slow mode while it is approaching encoder locations.
            if ((collector.getPower() > 0.01) && isCollectingIn)
            {
                collector.setPower(Constants.MOTOR_COLLECTOR_SLOW_IN);
            }
            else if ((collector.getPower() > 0.01) && !isCollectingIn)
            {
                collector.setPower(Constants.MOTOR_COLLECTOR_SLOW_OUT);
            }

            collectorLoopTimer.reset();
            // Wait until optical encoder reaches 1 of 4 positions.  Only do this loop if the motor
            // is powered and loop time is shorter than 2 seconds since we do not want to get stuck in it.
            if (collectorEncoderState = !channel.getState() && (Math.abs(collector.getPower()) > 0.01) && (timer.seconds() < 2))
            {
                /*time = getRuntime();
                telemetry.addData("Collector Channel: ", collectorEncoderState);
                telemetry.addData("Time", time);
                telemetry.update();*/
            }
            else
            {
                collector.setPower(0);
            }
        }
    }
}
