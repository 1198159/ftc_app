package org.firstinspires.ftc.team6220_2017;

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
        // todo Take this out when the arm is operational again
        isArmAttached = false;
        initializeRobot();

        waitForStart();
        // Move jewel servo so it is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();


        // Main loop
        while (opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // 1st driver:  left stick rotates robot, right stick translates robot
            // 2nd driver:  right bumper retracts jewel jostler if it falls
            driveMecanumWithJoysticks();
            // 2nd driver:  right stick moves arm
            //armMechanism.driveArm();
            // 1st driver:  dpad down collects, dpad up scores
            // 2nd driver:  a, b, x, and y raise the glyphter to 4 glyph heights
            glyphMechanism.driveGlyphMech();


            /*
             updates that need to happen each loop
             note:  eTime is not currently displayed (it interrupts other telemetry), but it may
             be useful later
            */
            //telemetry.addData("eTime:", eTime);
            updateCallback(eTime);
            telemetry.update();
            idle();
        }
    }
}
