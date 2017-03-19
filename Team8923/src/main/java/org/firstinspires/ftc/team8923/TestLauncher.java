package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Launcher Test", group = "Tests")
@Disabled
public class TestLauncher extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();
        waitForStart();

        waitForStart();

        while(opModeIsActive())
        {
            while(true)
            {
                // Drop collector so the hopper isn't blocked and run the collector backwards to help
                servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_UP.pos);
                motorCollector.setPower(-0.5);
                // Fire the first particle
                armCatapult();
                // Push second particle into the catapult
                servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
                // Stop the collector
                motorCollector.setPower(0.0);

                // Wait for the second particle to settle
                sleep(750);
                // Put the sweeper servo back
                servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
                // Launch second particle
                fireCatapult();
            }
        }
    }
}
