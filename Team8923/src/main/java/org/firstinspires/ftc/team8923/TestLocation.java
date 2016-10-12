package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/*
 * Used to test update location method
 */
@Autonomous(name = "Location Test", group = "Tests")
public class TestLocation extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        waitForStart();

        while(opModeIsActive())
        {
            updateRobotLocation();

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Angle", robotAngle);

            telemetry.update();
            idle();
        }
    }
}
