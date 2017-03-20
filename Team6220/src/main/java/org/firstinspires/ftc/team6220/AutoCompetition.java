package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 *  Autonomous allowing driver to select its routine
 */

@Autonomous(name = "AutoCompetition", group = "Autonomous")
@Disabled
public class AutoCompetition extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        //todo:  nullpointer exception
        runSetUp();
        initializeAuto();

        //sets starting location based on alliance
        if (alliance == Alliance.BLUE)
        {
            drive.robotLocation = new Transform2D(2.395, 0.210, 90.0);
            setRobotStartingOrientation(90.0);
        }
        else
        {
            drive.robotLocation = new Transform2D(0.210, 2.395, 0.0);
            setRobotStartingOrientation(0.0);
        }

        waitForStart();

        //delay is in seconds; pause takes milliseconds
        pause(delay * 1000);

        //Start tracking targets
        vuforiaHelper.startTracking();

        //todo remove position parameters from activateBeacon
        //not currently in use
        /*
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.LAUNCHANDBUTTONS)
        {
            drive.robotLocation = new Transform2D(0.210, 2.395, 180.0);
            setRobotStartingOrientation(180.0);

            //vuforia is not reliably available yet, so we must use encoders at first
            //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

            drive.moveRobot(0.25, 0.0, 180.0);
            pause(800);

            stopAllDriveMotors();

            //shoots a ball
            launcher.pullback();
            pauseWhileUpdating(3.0);
            launcher.loadParticle();
            pauseWhileUpdating(2.0);
            launcher.launchParticle();
            pauseWhileUpdating(2.0);
            launcher.pullBackMotor.setPower(0.0);

            drive.moveRobot(0.8, -0.5, 0.0);
            pause(1500);

            stopAllDriveMotors();

            turnTo(true, 0.0);

            stopAllDriveMotors();

            drive.moveRobot(0.0, 0.1, 0.0);
            pause(1200);

            //presses beacon 1
            pause(500);
            vuforiaAlign(false, true, 1.524, 0.0);

            stopAllDriveMotors();

            pause(500);
            activateBeacon(false, 1.524);

            drive.moveRobot(0.0, 0.1, 0.0);
            pause(2500);

            stopAllDriveMotors();
            //

            drive.moveRobot(0.0, -0.2, 0.0);
            pause(1000);

            stopAllDriveMotors();

            drive.moveRobot(0.25, 0.0, 0.0);
            pause(1200);

            stopAllDriveMotors();

            //presses beacon 2
            pause(1000);
            vuforiaAlign(false, true, 2.700, 0.0);

            stopAllDriveMotors();

            activateBeacon(false, 2.700);

            drive.moveRobot(0.0, 0.10, 0.0);
            pause(2000);

            drive.moveRobot(0.0, -0.5, 0.0);
            pause(3000);

            stopAllDriveMotors();
            //
        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.LAUNCH)
        {

        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.BUTTONS)
        {
            //vuforia is not reliably available yet, so we must use encoders at first
            //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

            drive.moveRobot(0.5, 1.0, 0.0);
            pause(1400);

            stopAllDriveMotors();

            turnTo(true, 0.0);

            stopAllDriveMotors();

            drive.moveRobot(0.0, 0.1, 0.0);
            pause(1500);

            //presses beacon 1
            pause(500);
            vuforiaAlign(false, true, 1.524, 0.0);

            stopAllDriveMotors();

            pause(500);
            activateBeacon(false, 1.524);

            drive.moveRobot(0.0, 0.1, 0.0);
            pause(2500);

            stopAllDriveMotors();
            //

            drive.moveRobot(0.0, -0.2, 0.0);
            pause(1000);

            stopAllDriveMotors();

            drive.moveRobot(0.25, 0.0, 0.0);
            pause(1200);

            stopAllDriveMotors();

            //presses beacon 2
            pause(1000);
            vuforiaAlign(false, true, 2.700, 0.0);

            stopAllDriveMotors();

            activateBeacon(false, 2.700);

            drive.moveRobot(0.0, 0.10, 0.0);
            pause(2000);

            drive.moveRobot(0.0, -0.5, 0.0);
            pause(3000);

            stopAllDriveMotors();
            //
        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.PARKANDCAPBALL)
        {

        }
        if (alliance == Alliance.RED && routineOption == RoutineOption.LAUNCHANDBUTTONS)
        {
            drive.robotLocation = new Transform2D(0.210, 2.395, 90.0);
            setRobotStartingOrientation(90.0);

            //vuforia is not reliably available yet, so we must use encoders at first
            //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

            drive.moveRobot(0.25, 0.0, 90.0);
            pause(800);

            stopAllDriveMotors();

            //shoots a ball
            launcher.pullback();
            pauseWhileUpdating(3.0);
            launcher.loadParticle();
            pauseWhileUpdating(2.0);
            launcher.launchParticle();
            pauseWhileUpdating(2.0);
            launcher.pullBackMotor.setPower(0.0);

            drive.moveRobot(0.8, 0.5, 90.0);
            pause(1500);

            stopAllDriveMotors();

            turnTo(true, 90.0);

            stopAllDriveMotors();

            drive.moveRobot(0.0, 0.1, 90.0);
            pause(1200);

            //presses beacon 1
            pause(500);
            vuforiaAlign(true, true, 1.524, 90.0);

            stopAllDriveMotors();

            pause(500);
            activateBeacon(true, 1.524);

            drive.moveRobot(0.0, 0.1, 90.0);
            pause(2500);

            stopAllDriveMotors();
            //

            drive.moveRobot(0.0, -0.2, 90.0);
            pause(1500);

            stopAllDriveMotors();

            drive.moveRobot(-0.3, 0.0, 90.0);
            pause(3000);

            //presses beacon 2
            pause(1000);
            vuforiaAlign(true, true, 2.700, 90.0);

            stopAllDriveMotors();

            activateBeacon(true, 2.700);

            drive.moveRobot(0.0, 0.10, 90.0);
            pause(2000);

            drive.moveRobot(0.0, -0.5, 90.0);
            pause(3000);

            stopAllDriveMotors();
            //
        }
        if (alliance == Alliance.RED && routineOption == RoutineOption.LAUNCH)
        {

        }
        if (alliance == Alliance.RED && routineOption == RoutineOption.BUTTONS)
        {
            //vuforia is not reliably available yet, so we must use encoders at first
            //navigateUsingEncoders(new Transform2D(1.524, 2.600, 90.0 - headingOffset));

            drive.moveRobot(-0.5, 1.0, 0.0);
            pause(1400);

            stopAllDriveMotors();

            turnTo(true, 90.0);

            stopAllDriveMotors();

            //presses beacon 1
            pause(1000);
            vuforiaAlign(true, true, 1.524, 90.0);

            drive.moveRobot(0.0, 0.2, 90.0);
            pause(1000);

            stopAllDriveMotors();

            activateBeacon(true, 1.524);

            drive.moveRobot(0.0, 0.1, 90.0);
            pause(2500);

            stopAllDriveMotors();
            //

            drive.moveRobot(0.0, -0.2, 90.0);
            pause(1500);

            stopAllDriveMotors();

            drive.moveRobot(-0.3, 0.0, 90.0);
            pause(3000);

            //presses beacon 2
            pause(1000);
            vuforiaAlign(true, true, 2.700, 90.0);

            stopAllDriveMotors();

            activateBeacon(true, 2.700);

            drive.moveRobot(0.0, 0.10, 90.0);
            pause(2000);

            drive.moveRobot(0.0, -0.5, 90.0);
            pause(3000);

            stopAllDriveMotors();
            //
        }
        if (alliance == Alliance.BLUE && routineOption == RoutineOption.PARKANDCAPBALL)
        {

        }
        */
    }
}
