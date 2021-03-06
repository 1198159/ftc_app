package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Encapsulates functionalities for the glyph-scoring mechanism on our robot.
 */

public class GlyphMechanism implements ConcurrentOperation
{
    // Declare preliminary variables------------------------------------------
    MasterOpMode op;

    private int[] glyphHeights;

    /*
     Since the glyphter motor's encoders are reset while it is deployed, using automatic
     functionality would normally run the glyphter into the ground.  Subtracting this value from
     the mechanism's target position allows us to retain automatic movement of the glyphter without
     retracting it before the start of TeleOp.
    */
    public int glyphterOffset = Constants.HEIGHT_3;

    // For debugging collector speeds
    private double motorCollectorCount = 0.5;

    // Tells us how long it has been since the last time the glyph mechanism was rotated
    private double turntableRotationTimer = 0;

    private boolean wasStickPressed = false;
    // Tells us whether or not the rotation mechanism has been rotated for collecting an additional
    // glyph
    private boolean isTurntableRotated = false;
    // Times the rotation mechanism so we can tell it when to stop moving
    private boolean isRotationTimerRunnning = false;
    //------------------------------------------------------------------------

    // We pass in MasterOpMode so that this class can access important functionalities such as
    // telemetry and pauseWhileUpdating
    public GlyphMechanism (MasterOpMode mode, int[] GlyphHeights)
    {
        this.op = mode;
        this.glyphHeights = GlyphHeights;
    }


    // todo REMEMBER:  Add hardware devices here
    public void initialize(HardwareMap hMap){}


    // Call at end of loop
    public void update(double eTime)
    {
        if (isRotationTimerRunnning)
        {
            turntableRotationTimer += eTime;

            // Check to see whether the timer has run out.  If so, we reset it and accomplish the
            // appropriate actions
            if (turntableRotationTimer > Constants.TURNTABLE_ROTATION_TIME)
            {
                // Give the rotation servo a small power to ensure the glyph mechanism doesn't move.
                // This power depends on which position the glyph mechanism is rotated to
                if (isTurntableRotated)
                    op.collectorTurntableServo.setPower(Constants.MINIMUM_TURNTABLE_POWER);
                else
                    op.collectorTurntableServo.setPower(-Constants.MINIMUM_TURNTABLE_POWER);

                turntableRotationTimer = 0;
                isRotationTimerRunnning = false;
            }
        }
    }


    // Rotates glyph mechanism in opposite directions based on whether it has been flipped or not
    public void rotateGlyphMech()
    {
        // Tell timer to start running at the end of the loop in which we call this method
        isRotationTimerRunnning = true;

        if (!isTurntableRotated)
        {
            op.collectorTurntableServo.setPower(1.0);

            isTurntableRotated = true;
        }
        else
        {
            op.collectorTurntableServo.setPower(-1.0);

            isTurntableRotated = false;
        }
    }


    // Takes input used to move all parts of the glyph mechanism
    public void driveGlyphMech()
    {
        // Note:  REMEMBER to restart robot if program is stopped, then adjust position manually
        // Glyphter controls---------------------------------------------------
         // Drive glyphter manually
        if (Math.abs(op.gamepad2.right_stick_y) >= Constants.MINIMUM_JOYSTICK_POWER)
        {
            wasStickPressed = true;
            op.motorGlyphter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Filter driver input to glyphter to increase ease of use
            double glyphterPower = op.stickCurve.getOuput(-op.gamepad2.right_stick_y);

            op.motorGlyphter.setPower(glyphterPower);
            op.telemetry.addData("rightStickY: ", glyphterPower);
            op.telemetry.update();
        }
         // Check to see if the joystick was just released.  If so, transition into automatic control
        else if (Math.abs(op.gamepad2.right_stick_y) < Constants.MINIMUM_JOYSTICK_POWER && wasStickPressed)
        {
            wasStickPressed = false;
            op.motorGlyphter.setPower(0);
            op.motorGlyphter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
         // Drive glyphter automatically
        else if (Math.abs(op.gamepad2.right_stick_y) < Constants.MINIMUM_JOYSTICK_POWER && !wasStickPressed)
        {
            // If autonomous was not run beforehand, this gives the driver the ability to remove
            // the effects of glyphterOffset
            if (op.driver2.isButtonJustPressed(Button.LEFT_STICK_PRESS) && op.driver2.isButtonJustPressed(Button.RIGHT_STICK_PRESS))
            {
                glyphterOffset = 0;
            }

            // Protect against Start + A / B running glyph mechanism into ground
            if (op.driver2.isButtonJustPressed(Button.A) && !op.driver2.isButtonPressed(Button.START))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[0] - glyphterOffset);
                op.motorGlyphter.setPower(1.0);
            }
            else if (op.driver2.isButtonJustPressed(Button.B) && !op.driver2.isButtonPressed(Button.START))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[1] - glyphterOffset);
                op.motorGlyphter.setPower(1.0);
            }
            else if (op.driver2.isButtonJustPressed(Button.Y))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[2] - glyphterOffset);
                op.motorGlyphter.setPower(1.0);
            }
            else if (op.driver2.isButtonJustPressed(Button.X))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[3] - glyphterOffset);
                op.motorGlyphter.setPower(1.0);
            }
            // Stow glyph mechanism
            else if (op.driver2.isButtonJustPressed(Button.START))
            {
                op.motorGlyphter.setTargetPosition(-glyphterOffset);
                op.motorGlyphter.setPower(1.0);
            }
        }
        //----------------------------------------------------------------------


        // Collector controls------------------------------------------------
         // Collect glyphs
        if (op.driver1.isButtonJustPressed(Button.DPAD_DOWN))
        {
            op.motorCollectorRight.setPower(0.8);
            op.motorCollectorLeft.setPower(0.8);
        }
         // Score glyphs
        else if (op.driver1.isButtonJustPressed(Button.DPAD_UP))
        {
            op.motorCollectorRight.setPower(-0.8);
            op.motorCollectorLeft.setPower(-0.8);

        }
         // Stack glyphs (need a slower speed for this)
        else if (op.driver1.isButtonJustPressed(Button.DPAD_RIGHT))
        {
            op.motorCollectorRight.setPower(-0.4);
            op.motorCollectorLeft.setPower(-0.4);

        }
         // Stop collector
        else if (op.driver1.isButtonJustPressed(Button.DPAD_LEFT))
        {
            op.motorCollectorRight.setPower(0);
            op.motorCollectorLeft.setPower(0);
        }
        // Allow adjustment of collection speeds if necessary (usually for debugging)
        else if (op.driver1.isButtonJustPressed(Button.LEFT_STICK_PRESS))
        {
            motorCollectorCount += 0.05;
            op.motorCollectorLeft.setPower(motorCollectorCount);
            op.motorCollectorRight.setPower(motorCollectorCount);
        }
        else if (op.driver1.isButtonJustPressed(Button.RIGHT_STICK_PRESS))
        {
            motorCollectorCount -= 0.05;
            op.motorCollectorLeft.setPower(motorCollectorCount);
            op.motorCollectorRight.setPower(motorCollectorCount);
        }
        //---------------------------------------------------------------------


        // Turntable controls-----------------------------------------
        if (op.driver2.isButtonJustPressed(Button.RIGHT_BUMPER))
        {
            rotateGlyphMech();
        }
         // If turntable servo starts malfunctioning, driver 2 can stop it.
        else if (op.driver2.isButtonJustPressed(Button.Y))
        {
            op.collectorTurntableServo.setPower(0);
        }
        //---------------------------------------------------------------------


        op.telemetry.addData("Glyphter Enc: ", op.motorGlyphter.getCurrentPosition());
        op.telemetry.addData(" MotorCollectorCount: ", motorCollectorCount);
        op.telemetry.update();
    }


    // todo Check to make sure this works properly
    // Similar to runToPosition, but uses a specialized PID loop for the glyphter.  Using runToPosition
    // can cause the glyphter motor to stall before reaching its target position
    public void driveGlyphterToPosition(int targetPosition, double maxPower)
    {
        double glyphterDiff = targetPosition - op.motorGlyphter.getCurrentPosition();
        double glyphterPower;

        // Check to see whether the glyphter is close enough to its target to stop moving
        while ((Math.abs(glyphterDiff) > Constants.GLYPHTER_TOLERANCE_TICKS) && op.opModeIsActive())
        {
            glyphterDiff = targetPosition - op.motorGlyphter.getCurrentPosition();

            // Transform glyphterDiff to motor power using PID
            op.glyphterFilter.roll(glyphterDiff);
            glyphterPower = op.glyphterFilter.getFilteredValue();

            // Ensure glyphter doesn't ever move faster than we want it to
            if (Math.abs(glyphterPower) > maxPower)
            {
                glyphterPower = Math.signum(glyphterPower) * maxPower;
            }

            // Ensure glyphter doesn't approach target position too slowly
            if (Math.abs(glyphterPower) < Constants.MINIMUM_GLYPHTER_POWER)
            {
                glyphterPower = Math.signum(glyphterPower) * Constants.MINIMUM_GLYPHTER_POWER;
            }

            op.motorGlyphter.setPower(glyphterPower);

            //op.telemetry.addData("Glyphter Encoder Value: ", op.motorGlyphter.getCurrentPosition());
            // Note:  This gives the ABSOLUTE VALUE of the motor power
            //op.telemetry.addData("Glyphter Power: ", op.motorGlyphter.getPower());
            op.telemetry.update();
        }

        op.motorGlyphter.setPower(0);
    }
}
