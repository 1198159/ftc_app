package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 *  Autonomous allowing driver to select its routine
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeAuto();

        if (alliance == Alliance.BLUE)
        {
            drive.robotLocation = new Transform2D(2.395, 0.210, 90.0);
            setRobotStartingOrientation(90.0);
            beaconActivationAngle = 0.0;
        }
        else
        {
            drive.robotLocation = new Transform2D(0.210, 2.395, 0.0);
            setRobotStartingOrientation(0.0);
            beaconActivationAngle = 90.0;
        }

        runSetUp();

        waitForStart();

        //Start tracking targets
        vuforiaHelper.startTracking();

        pause(delay);

        if (alliance == Alliance.BLUE && routineOption == RoutineOption.LAUNCHANDBUTTONS)
        {

        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.LAUNCH)
        {

        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.BUTTONS)
        {

        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.PARKANDCAPBALL)
        {

        }
        if (alliance == Alliance.RED && routineOption == RoutineOption.LAUNCHANDBUTTONS)
        {

        }
        if (alliance == Alliance.RED && routineOption == RoutineOption.LAUNCH)
        {

        }
        if (alliance == Alliance.RED && routineOption == RoutineOption.BUTTONS)
        {

        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.PARKANDCAPBALL)
        {

        }
    }
}
