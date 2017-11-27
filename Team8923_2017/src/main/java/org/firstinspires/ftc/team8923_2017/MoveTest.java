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
        initVuforia();//Initializes Vuforia
        waitForStart();

        double referenceAngle =  imu.getAngularOrientation().firstAngle;
        runtime.reset(); // used for timeout
        targetAngle = 0;
        maxSpeed = 0.2;
        saturationValue = 0.5;
        kAngle = 0.009;
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            angleError = currentRobotAngle - referenceAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;


            motorPowerFL = 0.2 + pivot;
            motorPowerFR = -0.2 + pivot;
            motorPowerBL = 0.2 + pivot;
            motorPowerBR = -0.2 + pivot;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);

            Color.RGBToHSV((sensorTopLeft.red() * 255) / 800, (sensorTopLeft.green() * 255) / 800, (sensorTopLeft.blue() * 255) / 800, hsvValuesTopLeft);
            Color.RGBToHSV((sensorTopRight.red() * 255) / 800, (sensorTopRight.green() * 255) / 800, (sensorTopRight.blue() * 255) / 800, hsvValuesTopRight);
            Color.RGBToHSV((sensorBottomRight.red() * 255) / 800, (sensorBottomRight.green() * 255) / 800, (sensorBottomRight.blue() * 255) / 800, hsvValuesBottomRight);

            telemetry.addData("TopRight_S: ", hsvValuesTopRight[1]);
            //telemetry.addData("V: ", hsvValuesTopLeft[2]);
            telemetry.addData("TopLeft_S: ", hsvValuesTopLeft[1]);
            telemetry.addData("BottomRight_S: ", hsvValuesBottomRight[1]);
            telemetry.update();
            idle();
        }
        while ((opModeIsActive()) && (hsvValuesTopRight[1] <= saturationValue) && (hsvValuesTopLeft[1] <= saturationValue) && (hsvValuesBottomRight[1] <= saturationValue));
        //
        stopDriving();

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
