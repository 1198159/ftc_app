package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Moves straight forward and launches two particles into the center vortex.
    Set up aligned with wall and with launcher pointed toward center vortex
*/
@Autonomous(name="Launch Two Particles to Center", group="6220")
public class AutoLaunchParticle extends MasterTeleOp
{
    double taskElapsedTime = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        pauseWhileUpdating(1.0);
        drive.moveRobot(-0.5, 0.0, 0.0);
        pauseWhileUpdating(1.0);
        drive.writeToMotors(new double[]{0.0, 0.0, 0.0, 0.0});
        pauseWhileUpdating(0.5);
        launcher.pullback();
        pauseWhileUpdating(3.0);
        launcher.launchParticle();
        pauseWhileUpdating(5.0);

    }
    void pauseWhileUpdating(double time)
    {
        while(opModeIsActive() && time > 0)
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();
            time -= eTime;

            telemetry.addData("Time Remaining:", time);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }

    }
}