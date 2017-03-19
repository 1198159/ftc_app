package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *  Tests the motors are properly hooked up to encoders etc. by giving them all the same power
 */

@Autonomous(name = "TestMotorsConstantPower", group = "Autonomous")
//@Disabled
public class TestMotorsConstantPower extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initializeHardware();

        waitForStart();

        ElapsedTime duration = new ElapsedTime();

        double powers[] = new double[4];
        powers[0] = 0.25;
        powers[1] = 0.25;
        powers[2] = 0.25;
        powers[3] = 0.25;

        drive.writeToMotors(powers);

        while (opModeIsActive())
        {
            idle();
        }
        stopAllDriveMotors();
    }
}
