package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        while (!isStarted())
        {
            // select driver 1
            if (gamepad1.x) driveSlater = true; // press x if Slater is driving
            if (gamepad1.y) driveSlater = false; // press y if Flynn is driving

            if (driveSlater) telemetry.addData("Driver: ", "Slater");
            else telemetry.addData("Driver: ", "Flynn");

            telemetry.update();
            idle();
        }

        super.initializeHardware();

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Init:", "Done");
        telemetry.update();
        waitForStart();
        curGGPos = motorGlyphGrab.getCurrentPosition(); // 0 is open (closing is positive, opening is negative)
        while (opModeIsActive())
        {
            //imuOmniTeleOp();
            omniDriveTeleOp();
            runJJ();
            runManualGlyphLift();
            runAutoGlyphLift();
            runManualGG();
            runAutoGG();
            updateTelemetry();

            // TODO: add some telemetry to display the motor power
            idle();
        }
    }
}
