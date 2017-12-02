package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Encapsulates functionalities for the glyph-scoring mechanism on our robot.
 */

public class GlyphMechanism
{
    MasterOpMode op;

    int[] glyphHeights;

    boolean manualControl = false;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public GlyphMechanism (MasterOpMode mode, int[] GlyphHeights)
    {
        this.op = mode;
        this.glyphHeights = GlyphHeights;
    }

    // Takes input used to move all parts of the glyph mechanism
    public void driveGlyphMech()
    {
        // Glyphter controls---------------------------------------------------
         // Checks bumpers to see if 2nd driver wants manual or automatic control
        if (op.driver2.isButtonJustPressed(Button.LEFT_BUMPER))
        {
            manualControl = false;
            op.motorGlyphter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (op.driver2.isButtonJustPressed(Button.RIGHT_BUMPER))
        {
            manualControl = true;
            op.motorGlyphter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        // Note:  REMEMBER to restart robot if program is stopped, then position is adjusted manually
         // Drive glyphter automatically
        if (!manualControl)
        {
            // Raise the glyphter to different positions corresponding to the buttons a, b, y, and x
            if (op.driver2.isButtonJustPressed(Button.A))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[0]);
                op.motorGlyphter.setPower(1.0);
            }
            else if (op.driver2.isButtonJustPressed(Button.B))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[1]);
                op.motorGlyphter.setPower(1.0);
            }
            else if (op.driver2.isButtonJustPressed(Button.Y))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[2]);
                op.motorGlyphter.setPower(1.0);
            }
            else if (op.driver2.isButtonJustPressed(Button.X))
            {
                op.motorGlyphter.setTargetPosition(glyphHeights[3]);
                op.motorGlyphter.setPower(1.0);
            }
            // Stow glyph mechanism
            else if (op.driver2.isButtonJustPressed(Button.START))
            {
                op.motorGlyphter.setTargetPosition(0);
                op.motorGlyphter.setPower(1.0);
            }
        }
         // Drive glyphter manually
        else
        {
            op.motorGlyphter.setPower(op.gamepad2.right_stick_y);
            op.telemetry.addData("rightStickY: ", op.gamepad2.right_stick_y);
        }
        //----------------------------------------------------------------------



        // Collector controls------------------------------------------------
         // Collect glyphs
        if (op.driver2.isButtonJustPressed(Button.DPAD_DOWN))
        {
            op.motorCollectorLeft.setPower(-0.5);
            op.motorCollectorRight.setPower(0.5);
        }
         // Score glyphs
        else if (op.driver2.isButtonJustPressed(Button.DPAD_UP))
        {
            op.motorCollectorLeft.setPower(0.5);
            op.motorCollectorRight.setPower(-0.5);
        }
         // Stop collector
        else if (op.driver2.isButtonJustPressed(Button.DPAD_LEFT))
        {
            op.motorCollectorLeft.setPower(0);
            op.motorCollectorRight.setPower(0);
        }
        //---------------------------------------------------------------------


        op.telemetry.addData("manualControl: ", manualControl);
        op.telemetry.addData("Glyphter Enc: ", op.motorGlyphter.getCurrentPosition());
        op.telemetry.update();
    }
}
