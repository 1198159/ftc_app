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
    boolean GGFlipped = false;
    //Declare variables here
    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        //initVuforia();//Initializes Vuforia
        waitForStart();

        motorGG.setTargetPosition(motorGG.getCurrentPosition() + 1100);
        motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 55.0), 1.0));

        sleep(400);
        if (!GGFlipped)
        {
            motorFF.setTargetPosition(FFZero + 750);
            motorFF.setPower(0.4);
            sleep(500);
            GGFlipped = true;
        }
        else
        {
            motorFF.setTargetPosition(FFZero);
            motorFF.setPower(-0.4);
            sleep(500);
            GGFlipped = false;
        }
        sleep(200);
        motorGG.setTargetPosition(motorGG.getCurrentPosition() - 950);
        motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 55.0), 1.0));
        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
