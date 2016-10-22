package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Used to test and debug autonomous methods
 */
@TeleOp(name = "Location Test", group = "Tests")
public class TestLocation extends MasterAutonomous
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        robotX = 0;
        robotY = 0;
        robotAngle = 0;
        resetHeadingOffset();

        waitForStart();

        vuforiaLocator.startTracking();

        while(opModeIsActive())
        {
            driveMecanumTeleOp();

            updateRobotLocation();

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Angle", robotAngle);

            sendTelemetry();
            idle();
        }
    }

    void driveMecanumTeleOp()
    {
        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        driveMecanum(angle, power, turnPower);
    }
}
