package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


/**
 * A test program used to try out any new ideas.
 */
@TeleOp(name="TeleOp TestBed", group = "6220")
//@Disabled
public class TeleOpTestBed extends MasterAutonomous
{
    VuforiaHelper vuforiaHelper = new VuforiaHelper();

    @Override public void runOpMode() throws InterruptedException
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);


        initializeRobot();

        waitForStart();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();

        // Main loop
        while(opModeIsActive())
        {
            if(driver1.isButtonJustPressed(Button.A))
            {
                turnTo(35,1.0);
            }
            if(driver1.isButtonJustPressed(Button.DPAD_UP))
            {
                motorCollector.setPower(0.9);
            }
            if(driver1.isButtonJustPressed(Button.DPAD_DOWN))
            {
                motorCollector.setPower(-0.9);
            }
            if(driver1.isButtonJustPressed(Button.DPAD_LEFT))
            {
                motorCollector.setPower(0.0);
            }
            if(driver1.isButtonJustPressed(Button.B))
            {
                servoMarker.setPosition(Constants.SERVO_MARKER_DEPLOYED);
                pauseWhileUpdating(0.5);
                servoMarker.setPosition(Constants.SERVO_MARKER_RETRACTED);
            }

            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

                //motorBL.setPower(1);
                //motorBR.setPower(1);
                //motorFL.setPower(1);
                //motorFR.setPower(1);

            telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }

}
