package org.firstinspires.ftc.team6220_2017;

/**
 * Encapsulates functionalities for the glyph-scoring mechanism on our robot
 */

public class GlyphMechanism
{
    MasterOpMode masterOpMode;

    // We pass in MasterOpMode so that this class can access important functionalities such as telemetry
    public GlyphMechanism (MasterOpMode mode)
    {
        this.masterOpMode = mode;
    }

    // Takes input used to move all parts of the glyph mechanism
    public void driveGlyphMech()
    {
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


    }
}
