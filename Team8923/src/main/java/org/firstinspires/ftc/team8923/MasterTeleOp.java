package org.firstinspires.ftc.team8923;

/**
 * This class contains all objects and methods that should be accessible by all TeleOpModes
 */
abstract class MasterTeleOp extends Master
{
    private int catapultTarget = 0;

    void driveMecanumTeleOp()
    {
        // Set slow mode if desired. 4 seems to be a good divisor for TeleOp
        if(gamepad1.dpad_down)
            slowModeDivisor = 3.0;
        else if(gamepad1.dpad_up)
            slowModeDivisor = 1.0;

        // Reverse drive if desired
        if(gamepad1.start)
            reverseDrive(false);
        if(gamepad1.back)
            reverseDrive(true);

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        driveMecanum(angle, power, turnPower);
    }

    // Extends and retracts beacon pusher
    void controlBeaconPusher()
    {
        if(gamepad1.a)
            servoBeaconPusher.setPosition(ServoPositions.BEACON_EXTEND.pos);
        else if(gamepad1.x)
            servoBeaconPusher.setPosition(ServoPositions.BEACON_RETRACT.pos);
    }

    // Runs lift up and down
    void runLift()
    {
        if(gamepad2.dpad_up)
            motorLift.setPower(1.0);
        else if(gamepad2.dpad_down)
            motorLift.setPower(-1.0);
        // If no button is pressed, stop!
        else
            motorLift.setPower(0);
    }

    // Runs collector. Can be run forwards and backwards
    void runCollector()
    {
        // Full speed is too fast
        double speedFactor = 0.6;
        motorCollector.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * speedFactor);
    }

    void controlCatapult()
    {
        // TODO: Add a touch sensor to ensure the catapult is in sync
        if(gamepad2.x && !motorCatapult.isBusy())
        {
            // 1680 is the number of ticks per revolution fo the output shaft of a NeveRest 60 gearmotor
            // We are using two of these revolutions because the motor is geared 3 to 1
            catapultTarget += 1680 * 3;
            motorCatapult.setTargetPosition(catapultTarget);
            motorCatapult.setPower(1.0);
        }
    }
}
