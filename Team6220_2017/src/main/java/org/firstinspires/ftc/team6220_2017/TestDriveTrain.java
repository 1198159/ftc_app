package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode for running robot without arm
 */

@TeleOp(name = "Test Drive Train")

public class TestDriveTrain extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        isArmAttached = false;
        initialize();

        waitForStart();

        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        while (opModeIsActive())
        {
            //finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            driveMecanumWithJoysticks();

            /*
             updates that need to happen each loop
             note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);

            telemetry.addData("FL", motorFL.getCurrentPosition());
            telemetry.addData("FR", motorFR.getCurrentPosition());
            telemetry.addData("BL", motorBL.getCurrentPosition());
            telemetry.addData("BR", motorBR.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
}
