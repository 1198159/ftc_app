package org.firstinspires.ftc.team8923_2017;

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
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            angleError = currentRobotAngle - referenceAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            //Sets values for motor power
            /*
            motorPowerFL = -0.2 + pivot;
            motorPowerFR = 0.2 + pivot;
            motorPowerBL = -0.2 + pivot;
            motorPowerBR = 0.2 + pivot;
            */
            motorPowerFL = -0.3 + pivot;
            motorPowerFR = 0.3 + pivot;
            motorPowerBL = -0.3 + pivot;
            motorPowerBR = 0.3 + pivot;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);

            idle();
        }
        while ((opModeIsActive()) && (hsvValuesTopRight[1] < saturationValue) && (hsvValuesTopLeft[1] < saturationValue) && (hsvValuesBottomRight[1] < saturationValue));

        //Stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);

        while (opModeIsActive())
        {
            //Run();
            idle();
        }

    }
}
