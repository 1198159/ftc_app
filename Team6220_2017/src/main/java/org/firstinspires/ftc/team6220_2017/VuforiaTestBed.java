package org.firstinspires.ftc.team6220_2017;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Created by Cole Welch on 10/1/2017.
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
public class VuforiaTestBed extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        //vuforiaHelper.startTracking();

        waitForStart();
    }

    //todo modify for jewels rather than beacons
    //we use this function to determine the color of jewels and knock them
    public void knockJewel (boolean redSide) throws InterruptedException
    {
        /*
        Color.colorToHSV(vuforiaHelper.getPixelColor(-127, 92, 0), );
        Color.colorToHSV(vuforiaHelper.getPixelColor(127, 92, 0), colorRightSide);
        */
    }
}
