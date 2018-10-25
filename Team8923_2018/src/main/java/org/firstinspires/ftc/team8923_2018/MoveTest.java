package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Move Test", group = "Test")
/*
 * Runable shell for Master Autonomous code
 */
//@Disabled
public class MoveTest extends MasterAutonomous
{
    //Declare variables here
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();
        //dankUnderglow(-0.5);
        waitForStart();
        moveLift(4500);
        moveAuto(80, 0, 0.35, 0.2, 3.0);
        //telemetry.addData("running", "running");
        //telemetry.update();
        while (opModeIsActive())
        {

        }
    }
}
