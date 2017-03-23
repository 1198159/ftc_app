package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Launcher Test", group = "Tests")
//@Disabled
public class TestLauncher extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();
        waitForStart();

        waitForStart();

        //while(opModeIsActive())
        {

            // Start : 3 particles, one in cup, two in hopper

            //Fire particle
            armCatapult();
            fireCatapult();

            //Load first particle from hopper
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_FIRST.pos);
            armCatapult();

            //fire particle
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
            fireCatapult();

            //Load second particle from hopper
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
            armCatapult();

            //Fire particle, run the collector
            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
            fireCatapult();
            motorCollector.setPower(1);

            //arm, load, and fire fourth particle
            sleep(300);

            servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
            armCatapult();
            fireCatapult();


            /*while(true)
            {
                // Drop collector so the hopper isn't blocked and run the collector backwards to help
                servoCollectorHolder.setPosition(ServoPositions.COLLECTOR_HOLDER_UP.pos);
                //motorCollector.setPower(-0.5);
                // Fire the first particle
                armCatapult();
                // Push second particle into the catapult
                servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_PUSH_SECOND.pos);
                // Stop the collector
                //motorCollector.setPower(0.0);

                // Wait for the second particle to settle
                sleep(750);
                // Put the sweeper servo back
                servoHopperSweeper.setPosition(ServoPositions.HOPPER_SWEEP_BACK.pos);
                // Launch second particle
                fireCatapult();
            }*/
        }
    }
}
