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
            GoldLocation position = landAndDetectMineral();
            switch (position)
            {
                case LEFT:
                    knockOffLeftMineral();
                    break;
                case CENTER:
                    knockOffCenterMineral();
                    break;
                case RIGHT:
                    knockOffRightMineral();
                    break;
            }
            idle();
        }

    }
}
