package org.firstinspires.ftc.team8923_2017;

import android.graphics.Color;
import android.hardware.Camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name="IMU Movement Test", group = "Test")
/*
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest extends MasterAutonomous
{
    //Declare variables here
    @Override
    public void runOpMode() throws InterruptedException
    {
        InitHardware();
        //ChooseOptions();
        InitAuto();
        waitForStart();
        while (opModeIsActive())
        {
            Run();
        }
        /*while (opModeIsActive())
        {
            telemetry.addData("IMU first angle", imu.getAngularOrientation().firstAngle);
            telemetry.addData("IMU second angle", imu.getAngularOrientation().secondAngle);
            telemetry.addData("IMU third angle", imu.getAngularOrientation().thirdAngle);
            telemetry.update();
        }*/
    }
}
