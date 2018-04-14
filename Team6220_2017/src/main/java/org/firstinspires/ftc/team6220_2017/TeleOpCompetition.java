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
        isArmAttached = true;
        initializeRobot();

        waitForStart();
        // Move jewel servos so jewel jostler is out of the way of the glyph mechanism
        verticalJewelServoToggler.retract();
        lateralJewelServo.setPosition(Constants.LATERAL_JEWEL_SERVO_NEUTRAL);

        // Accounts for delay between initializing the program and starting TeleOp
        lTime = timer.seconds();


        // Main loop
        while (opModeIsActive())
        {
            // Finds the time elapsed each loop
            double eTime = timer.seconds() - lTime;
            lTime = timer.seconds();

            // 1st driver:  left stick rotates robot, right stick translates robot, right bumper
            // toggles slow mode
            driveMecanumWithJoysticks();
            // 2nd driver:  left stick moves arm, triggers move wrist, dpad up toggles grabber
            armMechanism.driveArm();
            // 1st driver:  dpad down collects, dpad up scores, dpad left stops collector, dpad
            // right ejects glyphs slowly
            // 2nd driver:  a, b, x, y raise the glyphter to 4 glyph heights, right stick controls
            // glyphter manually, start returns glyphter to initial position
            glyphMechanism.driveGlyphMech();


            // Extra functionalities-------------------------------------------------------
             // For testing and in case glyph clip needs to be actuated during TeleOp
            if (driver1.isButtonJustPressed(Button.B))
            {
                glyphClipServoToggler.toggle();
                telemetry.addData("Glyph Clip Pos:", glyphClipServo.getPosition());
            }

             // Allows the 2nd driver to raise the vertical jewel servo if it fails during the match
            if(driver2.isButtonPressed(Button.LEFT_BUMPER))
            {
                verticalJewelServoToggler.retract();
            }

             // 1st driver can use this to quickly get ready to collect after delivering glyphs
            if (driver1.isButtonJustPressed(Button.LEFT_BUMPER))
            {
                autoAlignToScore();
            }
            //-----------------------------------------------------------------------------


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
