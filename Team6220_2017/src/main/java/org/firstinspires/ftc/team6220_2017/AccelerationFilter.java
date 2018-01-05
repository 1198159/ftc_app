package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Limits the rate at which a motor can accelerate.  This
 * decreases the likelihood that the robot's wheels will
 * slip when driving.  Can be implemented in autonomous navigation
 * and to improve manual control of the robot.
 */

public class AccelerationFilter implements Filter
{
    MasterOpMode op;

    // Initialize acceleration and deceleration rates
    // Units are motor power per millisecond
    double maxPowPerMilliIncrease = 1.0;
    double maxPowPerMilliDecrease = 1.0;

    //variables to keep track of the last time we were called.
    ElapsedTime timer;
    double millisecondsWhenLastCalled;

    // Stores new value and last value to be used in filter
    // Old value has index 1 and new value has index 0
    public double[] values = new double[2];

    //If we haven't been called in a really long time, here's a max value of elapsed time for use in our calculations
    //so the robot doesn't think it can accelerate far more than it should.
    //This value can be tuned, since larger values will allow the first iteration to accelerate/decelerate more.
    //If this value is too small, however, it will always be used instead of the actual elapsed time.
    final private int MAX_LOOP_TIME_MS = 60;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public AccelerationFilter(MasterOpMode mode, double maxAccel, double maxDecel)
    {
        this.op = mode;
        this.maxPowPerMilliIncrease = maxAccel;
        this.maxPowPerMilliDecrease = maxDecel;
        timer = new ElapsedTime();
        millisecondsWhenLastCalled = 0.0;
    }


    // Update with new motor power
    public void roll(double newPower)
    {
        // Introduce new value
        values[1] = values[0];
        values[0] = newPower;
    }


    public double getFilteredValue()
    {
        final double millisecondsNow = timer.milliseconds();

        // Determine how much time has passed since the last time we were called---------------
        // If it's been a long time since this method was last called, reset all values
        double elapsedMilliseconds = millisecondsNow - millisecondsWhenLastCalled;

        if (elapsedMilliseconds > MAX_LOOP_TIME_MS)
        {
            elapsedMilliseconds = MAX_LOOP_TIME_MS;
            values[1] = 0;
        }
        //--------------------------------------------------------------------------------------

        // Determine how much the motor power has changed since last call
        final double changeInPower = values[0] - values[1];

        double allowableChange = 0.0;

        // Is the magnitude of the control remaining constant, increasing, or decreasing?
        if (values[1] == values[0])
        {
            // No change to control value since last call. We are neither accelerating nor decelerating
            allowableChange = 0.0;
            //op.telemetry.addData("case: ", "zero");
        }
        else if ( ((Math.signum(values[0]) > 0) & (values[1] < values[0])) ||
                ((Math.signum(values[0]) < 0) & (values[1] > values[0])) )
        {
            // Accelerating
            allowableChange = elapsedMilliseconds * maxPowPerMilliIncrease;
            // Check to see whether we should use our allowable change or the actual change.  If the
            // actual change is smaller than the allowable change, we should disregard the allowable
            // change
            allowableChange = Math.min(Math.abs(allowableChange), Math.abs(changeInPower)) * Math.signum(changeInPower);
            op.telemetry.addData("case: ", "accel");
        }
        else
        {
            // Decelerating
            allowableChange = elapsedMilliseconds * maxPowPerMilliDecrease;
            // See above
            allowableChange = Math.min(Math.abs(allowableChange), Math.abs(changeInPower)) * Math.signum(changeInPower);
            op.telemetry.addData("case: ", "decel");
        }

        final double filteredPower = values[1] + allowableChange;

        // Store timer value for comparison next loop
        millisecondsWhenLastCalled = millisecondsNow;

        return filteredPower;
    }
}
