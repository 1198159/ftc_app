package org.firstinspires.ftc.team8923_2017;

import android.graphics.Color;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="Move Test", group = "Swerve")
//@Disabled
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest extends MasterAutonomous
{
    private ElapsedTime runtime = new ElapsedTime();
    double targetAngle;
    double maxSpeed;
    double saturationValue;
    //Declare variables here
    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        //initVuforia();//Initializes Vuforia
        waitForStart();

        double referenceAngle =  imu.getAngularOrientation().firstAngle;
        move(referenceAngle, 3000, 0.0, 0.015, 0.35, 1.1);//Go towards parking spot//Was 2.15

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
