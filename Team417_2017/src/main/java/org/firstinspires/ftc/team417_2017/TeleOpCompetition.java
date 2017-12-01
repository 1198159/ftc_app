package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();
        telemetry.addData("Init:", "Done");
        telemetry.update();
        waitForStart();
        curGGPos = motorGlyphGrab.getCurrentPosition(); // 0 is open (closing is positive, opening is negative)
        maxGGPos = 10 * COUNTS_PER_GG_REV; // maxGGPos equals the # rev to open GG times 103 counts per rev
        while (opModeIsActive())
        {
            servoJewel.setPosition(JEWEL_INIT);
            //imuOmniTeleOp();
            omniDriveTeleOp();
            // TODO: add some telemetry to display the motor power
            idle();
        }
    }
}
