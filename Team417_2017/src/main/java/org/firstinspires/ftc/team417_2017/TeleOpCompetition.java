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
        while (opModeIsActive())
        {
            servoJewelStore.setPosition(JEWEL_STORE_INIT);
            servoJewelDrop.setPosition(JEWEL_DROP_INIT);
            imuOmniTeleOp();
            //omniDriveTeleOp();
            // TODO: add some telemetry to display the motor power
            idle();
        }
    }
}
