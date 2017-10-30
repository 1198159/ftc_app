package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Move Test", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia

        waitForStart();

        double referenceAngle =  imu.getAngularOrientation().firstAngle;
        MoveIMU(referenceAngle, -40.0, -90, 0.015, 0.5, 3.0);
        //move(-20, 0.5, 3.0);
        //move(-100, 0.5, 3.0);

        while (opModeIsActive())
        {
            telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
            telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
            telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
            telemetry.addData("EncoderBR", motorBR.getCurrentPosition());
            telemetry.update();
            //Run();
            idle();
        }

    }
}
