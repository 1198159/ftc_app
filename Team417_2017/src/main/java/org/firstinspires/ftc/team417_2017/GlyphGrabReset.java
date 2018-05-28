package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "GlyphGrab Reset")
public class GlyphGrabReset extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();
        telemetry.addData("Init:", "Done");
        telemetry.update();
        waitForStart();
        curGGPos = motorGlyphGrab.getCurrentPosition(); // 0 is open (closing is positive, opening is negative)
        while (opModeIsActive())
        {
            servoJewel.setPosition(JEWEL_INIT);
            if(gamepad2.right_bumper) // CLOSE (counter goes negative when closing)
            {
                motorGlyphGrab.setPower(0.05);
            }
            else if(gamepad2.left_bumper) // OPEN (counter goes positive when opening)
            {
                motorGlyphGrab.setPower(-0.05);
            }
            else // turn motor off
            {
                motorGlyphGrab.setPower(0.0);
            }
            idle();
        }
    }
}