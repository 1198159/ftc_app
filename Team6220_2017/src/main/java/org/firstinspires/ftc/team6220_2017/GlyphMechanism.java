package org.firstinspires.ftc.team6220_2017;

/**
 * Encapsulates functionalities for the glyph-scoring mechanism on our robot
 */

public class GlyphMechanism
{
    MasterOpMode op;

    int[] glyphHeights;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public GlyphMechanism (MasterOpMode mode, int[] GlyphHeights)
    {
        this.op = mode;
        this.glyphHeights = GlyphHeights;
    }

    // Takes input used to move all parts of the glyph mechanism
    public void driveGlyphMech()
    {
        // 1st driver controls------------------------------------------------
         // Collect glyphs
        if (op.driver2.isButtonJustPressed(Button.DPAD_DOWN))
        {
            op.motorCollectorLeft.setPower(-1.0);
            op.motorCollectorRight.setPower(1.0);
        }
         // Score glyphs
        else if (op.driver2.isButtonJustPressed(Button.DPAD_UP))
        {
            op.motorCollectorLeft.setPower(0.5);
            op.motorCollectorRight.setPower(-0.5);
        }
        //---------------------------------------------------------------------


        // 2nd driver controls-------------------------------------------------
         // Raise the glyphter to different positions corresponding to the buttons a, b, y, and x
        if (op.driver2.isButtonJustPressed(Button.A))
        {
            op.motorGlyphter.setTargetPosition(glyphHeights[0]);
            op.motorGlyphter.setPower(0.25);
        }
        else if (op.driver2.isButtonJustPressed(Button.B))
        {
            op.motorGlyphter.setTargetPosition(glyphHeights[1]);
            op.motorGlyphter.setPower(0.25);
        }
        else if (op.driver2.isButtonJustPressed(Button.Y))
        {
            op.motorGlyphter.setTargetPosition(glyphHeights[2]);
            op.motorGlyphter.setPower(0.25);
        }
        else if (op.driver2.isButtonJustPressed(Button.X))
        {
            op.motorGlyphter.setTargetPosition(glyphHeights[3]);
            op.motorGlyphter.setPower(0.25);
        }
         // Stow glyph mechanism
        else if (op.driver2.isButtonJustPressed(Button.START))
        {
            op.motorGlyphter.setTargetPosition(0);
            op.motorGlyphter.setPower(0.25);
        }
        //----------------------------------------------------------------------

        op.telemetry.addData("Glyphter Enc: ", op.motorGlyphter.getCurrentPosition());
        op.telemetry.update();
    }
}
