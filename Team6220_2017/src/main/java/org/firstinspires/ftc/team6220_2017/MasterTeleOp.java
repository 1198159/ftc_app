package org.firstinspires.ftc.team6220_2017;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input
*/
abstract public class MasterTeleOp extends MasterOpMode
{
    //                                                   y = 0.0 + 1/2x + 0.0 + 1/2x^3
    Polynomial stickCurve = new Polynomial(new double[]{ 0.0, 0.5, 0.0, 0.5 });

    //takes driver 1 stick input and uses it to move the robot
    void driveMecanumWithJoysticks()
    {
        double eTime = timer.seconds() - lTime;
        lTime = timer.seconds();

        //factor changing magnitude of vertical and horizontal movement
        double tFactor;
        //factor changing magnitude of rotational movement
        double rFactor;

        //slows down the robot if slow mode is requested
        // note: factors are different for translation and rotation
        if (driver1.isButtonPressed(Button.RIGHT_BUMPER))
        {
            tFactor = 0.2;
            rFactor = 0.4;
        }
        else
        {
            tFactor = 1.0;
            rFactor = 1.0;
        }

        //stick inputs must be changed from x and y to angle and drive power
        double angle = driver1.getRightStickAngle();
        double power = tFactor * stickCurve.getOuput(driver1.getRightStickMagnitude());
        double rotationPower = rFactor * stickCurve.getOuput(gamepad1.left_stick_x);

        driveMecanum(angle, power, rotationPower);

        telemetry.addData("eTime: ", eTime);
        telemetry.update();
    }
}
