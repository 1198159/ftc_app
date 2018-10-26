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
        initAuto();

        waitForStart();

        telemetry.clear();

        while (opModeIsActive())
        {
            int position = landAndDetectMineral();
            switch (position)
            {
                case -1:
                    knockOffLeftMineral();
                    break;
                case 0:
                    knockOffCenterMineral();
                    break;
                case 1:
                    knockOffRightMineral();
                    break;
            }
            idle();
        }

    }
}
