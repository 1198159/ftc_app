package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Encoder Test", group = "Swerve")
// @Disabled

public class EncoderTest extends MasterAutonomous
{
    int motorFLCount;
    int motorFRCount;
    int motorBLCount;
    int motorBRCount;

    public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        super.initializeHardware();
        telemetry.addData("initialization", "done");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        autoRuntime.reset(); // set the 30 second timer

        // Run the motors
        motorFL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBL.setPower(0.5);
        motorBR.setPower(0.5);

        sleep(5000); // run for 5 seconds

        // Stop the motors
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);

        // Read the motor encoders
        motorFLCount = motorFL.getCurrentPosition();
        motorFRCount = motorFR.getCurrentPosition();
        motorBLCount = motorBL.getCurrentPosition();
        motorBRCount = motorBR.getCurrentPosition();

        // display the encoder counts
        telemetry.addData("Encoder:", "FL: %d, FR: %d, BL: %d, BR: %d", motorFLCount, motorFRCount, motorBLCount, motorBRCount);
        telemetry.update();

        sleep(5000);
    }

}
