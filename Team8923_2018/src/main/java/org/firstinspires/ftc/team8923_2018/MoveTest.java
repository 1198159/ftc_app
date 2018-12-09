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
        robotAngle = 0;
        robotX = 0;
        robotY = 0;
        initAuto();
        headingOffset = 0;
        //dankUnderglow(-0.5);
        waitForStart();
        //moveLift(4500);
        //moveAuto(80, 0, 0.35, 0.2, 3.0);
        //telemetry.addData("running", "running");
        //telemetry.update();
        while (opModeIsActive())
        {
            driveToPoint(0, 500, 0);
            sleep(500);
            driveToPoint(-500, 500, 0);
            sleep(500);
            driveToPoint(-500, 0, 0);
            sleep(500);
            driveToPoint(0, 0, 0);
            sleep(50000);
        }
    }
}
