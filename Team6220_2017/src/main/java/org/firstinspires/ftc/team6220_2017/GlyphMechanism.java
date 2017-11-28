package org.firstinspires.ftc.team6220_2017;

/**
 * Encapsulates functionalities for the glyph-scoring mechanism on our robot
 */

public class GlyphMechanism
{
    MasterOpMode masterOpMode;

    int[] glyphHeights;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public GlyphMechanism (MasterOpMode mode, int[] GlyphHeights)
    {
        this.masterOpMode = mode;
        this.glyphHeights = GlyphHeights;
    }

    // Takes input used to move all parts of the glyph mechanism
    public void driveGlyphMech()
    {
        // 1st driver controls
         // Collect glyphs
        if (masterOpMode.driver1.isButtonJustPressed(Button.DPAD_DOWN))
        {
            masterOpMode.motorCollectorLeft.setPower(-1.0);
            masterOpMode.motorCollectorRight.setPower(1.0);
        }
         // Score glyphs
        else if (masterOpMode.driver1.isButtonJustPressed(Button.DPAD_UP))
        {
            masterOpMode.motorCollectorLeft.setPower(1.0);
            masterOpMode.motorCollectorRight.setPower(-1.0);
        }
        //

        // 2nd driver controls
         // Raise the glyphter to different positions corresponding to the buttons a, b, y, and x
        if (masterOpMode.driver2.isButtonJustPressed(Button.A))
            masterOpMode.motorGlyphter.setTargetPosition(glyphHeights[0]);
        else if (masterOpMode.driver2.isButtonJustPressed(Button.B))
            masterOpMode.motorGlyphter.setTargetPosition(glyphHeights[1]);
        else if (masterOpMode.driver2.isButtonJustPressed(Button.Y))
            masterOpMode.motorGlyphter.setTargetPosition(glyphHeights[2]);
        else if (masterOpMode.driver2.isButtonJustPressed(Button.X))
            masterOpMode.motorGlyphter.setTargetPosition(glyphHeights[3]);

         // Stow glyph mechanism
        if (masterOpMode.driver2.isButtonJustPressed(Button.BACK))
            masterOpMode.motorCollectorLeft.setTargetPosition(0);
        //
    }
}
