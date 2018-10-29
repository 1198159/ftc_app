package org.firstinspires.ftc.team8923_2018;

import android.annotation.TargetApi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name="Autonomous Competition", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class AutonomousCompetition extends MasterAutonomous
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        configureAutonomous();
        telemetry.update();
        initAuto();

        waitForStart();
        telemetry.clear();

        while (opModeIsActive())
        {
            GoldLocation position = landAndDetectMineral();
            moveAuto(0, 10, 0.2, 0.2, 3.0);//straighten out robot
            moveAuto(80, 0, 0.35, 0.2, 3.0);//move off of hook
            double referenceAngle =  imu.getAngularOrientation().firstAngle; // Get a reference ange from the IMU for future movements using IMU

            //NOTE this is only for depot side
            switch (startLocation)
            {
                case DEPOT:
                {
                    switch (position)
                    {
                        case LEFT:
                        {
                            moveAuto(0, -280, 0.5, 0.3, 3.0);
                            moveAuto(480, 0, 0.5, 0.3, 3.0);
                            moveAuto(0, -660, 0.5, 0.3, 3.0);
                            //End of knock off mineral phase
                            //Other phases to be decided later
                            imuPivot(referenceAngle, -45, 0.5, 0.015, 3.0);
                            moveAuto(0, -650, 0.5, 0.3, 3.0);
                            moveJJ(-1);
                            sleep(500);
                            moveAuto(0, 500, 1.0, 0.3, 3.0);
                            moveAuto(300, 0, 0.3, 0.3, 3.0);
                            moveAuto(0, 1600, 1.0, 0.3, 3.0);
                            moveAuto(0, 100, 0.3, 0.3, 3.0);
                            break;

                        }
                        case CENTER:
                        {
                            moveAuto(0, -1270, 0.5, 0.35, 3.0);
                            moveAuto(0, 75, 0.7, 0.35, 3.0);
                            //End of knock off mineral phase
                            //Other phases to be decided later
                            imuPivot(referenceAngle, -90, 0.5, 0.015, 2.0);//0.5
                            moveAuto(177, 0, 0.5, 0.35, 3.0);
                            moveJJ(-1);
                            sleep(500);
                            moveAuto(-150, 0, 0.5, 0.35, 3.0);
                            moveAuto(0, 250, 0.5, 0.35, 3.0);
                            imuPivot(referenceAngle, -225, -0.3, 0.015, 2.0);
                            moveAuto(0, 2050, 1.0, 0.35, 5.0);
                            break;
                        }
                        case RIGHT:
                        {
                            moveAuto(0, -280, 0.5, 0.3, 3.0);
                            moveAuto(-480, 0, 0.5, 0.3, 3.0);
                            moveAuto(0, -660, 0.5, 0.3, 3.0);
                            //End of knock off mineral phase
                            //Other phases to be decided later
                            break;
                        }
                    }
                }
                case CRATER:
                {

                }
            }

            idle();
            break;
        }

    }
}
