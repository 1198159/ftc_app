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
        initAuto();

        robotX = 0;
        robotY = 0;
        robotAngle = 0;

        waitForStart();

        vuforiaLocator.startTracking();

        while(opModeIsActive())
        {
            driveMecanumTeleOp();

            // Translate forward, backward, right, then left one tile to check accuracy. Should
            // only be used before driving because of absolute location
            if(gamepad1.a)
            {
                driveToPoint(609, 0, 0);
                sleep(2000);
                driveToPoint(0, 0, 0);
                sleep(2000);
                driveToPoint(0, -609, 0);
                sleep(2000);
                driveToPoint(0, 0, 0);
                sleep(2000);
            }
            // Turn left then right 90 degrees to check accuracy.
            else if(gamepad1.b)
            {
                turnToAngle(90);
                sleep(2000);
                turnToAngle(0);
                sleep(2000);
            }

            updateRobotLocation();

            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Angle", robotAngle);

            sendTelemetry();
            idle();
        }
    }

    // This is copied from MasterTeleOp, because we don't have access to it
    private void driveMecanumTeleOp()
    {
        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        driveMecanum(angle, power, turnPower);
    }
}
