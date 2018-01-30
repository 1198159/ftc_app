package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous Move Tests", group = "Swerve")
// @Disabled

public class AutonomousMoveTests extends MasterAutonomous
{

    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        super.initializeHardware();

        telemetry.addData("Path: ", "Init Done");
        telemetry.update();

// Wait for the game to start (driver presses PLAY)
        waitForStart();
        autoRuntime.reset(); // set the 30 second timer

// START OF AUTONOMOUS

        maxGGPos = -450;
        closeGG();
        sleep(500);
        raiseGM();
        sleep(500);
        openGG(-200);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();

    }
}
