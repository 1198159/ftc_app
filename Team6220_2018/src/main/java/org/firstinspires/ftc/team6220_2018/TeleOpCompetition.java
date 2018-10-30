package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 *  Uses driver input to operate robot during TeleOp period
 */

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{

    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeRobot();
        /*LynxModule myModule = hardwareMap.get(LynxModule.class, "Expansion Hub 2");
        myModule.setDebug(LynxModule.DebugGroup.MOTOR0, LynxModule.DebugVerbosity.HIGH);*/

        waitForStart();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();


        // Main loop
        while (opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // Drive controls
            driveMecanumWithJoysticks();
            driveHanger();

            if (driver1.isButtonJustPressed(Button.X))
                servo.setPosition(1.0);
            else if (driver1.isButtonJustPressed(Button.B))
                servo.setPosition(0);


            // Extra functionalities-------------------------------------------------------
            /*
             Updates that need to happen each loop
             Note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
