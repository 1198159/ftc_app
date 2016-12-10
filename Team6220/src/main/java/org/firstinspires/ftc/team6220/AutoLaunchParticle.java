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

    double[][] tasks = new double[][]
            {
                    //angle to drive  , speed , duration(sec)
                    {   Math.PI      ,  0.7  ,     1.2      },

            };
    int currentTask = 0;
    int numTasks = tasks.length;
    double taskElapsedTime = 0;
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        while (opModeIsActive())
        {
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            if(currentTask != numTasks)
            {
                taskElapsedTime += eTime;
                double[] command = SequenceUtilities.scalarMultiply(
                        Utility.normalizedComponentsFromAngle(tasks[currentTask][0]),
                        tasks[currentTask][1]);
                double[] powers = drive.getMotorPowersFromMotion(new Transform2D(command[0],command[1],0));
                drive.writeToMotors(powers);
                if (taskElapsedTime > tasks[currentTask][2])
                {
                    currentTask += 1;
                    taskElapsedTime = 0;
                }
            }

            telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}