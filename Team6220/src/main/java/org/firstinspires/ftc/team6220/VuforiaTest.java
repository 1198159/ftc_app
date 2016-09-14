package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcontroller.internal.VuforiaTracker;

import java.util.HashMap;

/*
    Test opmode that displays one value from a tracked object from vuforia.
*/

@TeleOp(name="Vuforia Test", group="Tests")
public class VuforiaTest extends LinearOpMode
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        //set up vuforia
        VuforiaTracker Tracker = FtcRobotControllerActivity.getVuforia();
        Tracker.addTrackables("BeaconImages.xml");
        Tracker.initVuforia();

        waitForStart();

        while(opModeIsActive())
        {
            //index data by tracked object
            HashMap<String, double[]> data = Tracker.getVuforiaData();

            //
            if(data.containsKey("lego"))
            {
                try
                {
                    telemetry.addData("|Lego| rX:", data.get("lego")[6]);
                }
                catch(Exception e)
                {
                    telemetry.addData("|Lego| rX:", "not found");
                }

            }

            telemetry.update();
        }

        try {
            Tracker.destroy();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

}
