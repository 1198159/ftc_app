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

        //moveLift(500);

        telemetry.clear();

        while (opModeIsActive())
        {
            driveToPoint(0,500, 0, 0.6);
            driveToPoint(-500,500, 0, 0.6);
            driveToPoint(-500,0, 0, 0.6);
            driveToPoint(0,0, 0, 0.6);
            idle();
        }

    }
}
