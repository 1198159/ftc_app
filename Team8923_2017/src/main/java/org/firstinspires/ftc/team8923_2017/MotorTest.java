package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Motor Test", group = "Swerve")
/**
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MotorTest extends MasterAutonomous
{
    //Declare variables here

    @Override
    public void runOpMode() throws InterruptedException
    {
        //ChooseOptions();

        InitAuto();//Initializes Hardware and sets position based on alliance
        initVuforia();//Initializes Vuforia

        waitForStart();

        pivotFixed(0.1);
        telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
        telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
        telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
        telemetry.addData("EncoderBR", motorBR.getCurrentPosition());
        telemetry.update();
        sleep(3000);
        pivotFixed(0.2);
        telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
        telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
        telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
        telemetry.addData("EncoderBR", motorBR.getCurrentPosition());
        telemetry.update();
        sleep(3000);
        pivotFixed(0.5);
        telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
        telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
        telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
        telemetry.addData("EncoderBR", motorBR.getCurrentPosition());
        telemetry.update();
        sleep(3000);
        pivotFixed(0.9);
        telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
        telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
        telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
        telemetry.addData("EncoderBR", motorBR.getCurrentPosition());
        telemetry.update();
        sleep(3000);

        /*
        pivotFixed(-0.1);
        sleep(1000);
        pivotFixed(-0.2);
        sleep(1000);
        pivotFixed(-0.5);
        sleep(1000);
        pivotFixed(-0.9);
        */


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
