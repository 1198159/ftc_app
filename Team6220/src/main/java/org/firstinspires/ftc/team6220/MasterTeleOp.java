package org.firstinspires.ftc.team6220;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input
*/
abstract public class MasterTeleOp extends MasterOpMode
{
    //TODO test and use FIR filtered input for drive
    //                                                         y= 1/400x^2-x+100 @ x in {0,200}
    //                                                         Gradual smooth concave up curve
    //                                                         between 100 and 0.
    FIRFilter pilotInputFilter = new FIRFilter(new Polynomial(new double[]{100, -1, 0.0025}), 200);

    //                                                  y = 0.0 + 1/2x + 0.0 + 1/2x^3
    Polynomial pilotInputCurve = new Polynomial(new double[]{ 0.0, 0.5, 0.0, 0.5 });

    //takes driver 1 stick input and uses it to move the robot
    void driveRobotWithJoysticks(double xMotionAxis, double yMotionAxis, double rotationAxis)
    {
        double eTime = timer.seconds() - lTime;
        lTime = timer.seconds();

        //factor changing magnitude of vertical and horizontal movement
        double tFactor;
        //factor changing magnitude of rotational movement
        double rFactor;

        //slows down the robot if slow mode is requested
        if(driver1.isButtonPressed(Button.RIGHT_BUMPER))
        {
            tFactor = 0.2;
            rFactor = 0.4;
        }
        else
        {
            tFactor = 1.0;
            rFactor = 1.0;
        }

        //logic that senses whether the driver is attempting to turn.  If he is not, the robot
        //adjusts itself to ensure its heading is correct
        if (false)// (Math.abs(pilotInputCurve.getOuput(rotationAxis) * rFactor) < Constants.MINIMUM_TURNING_POWER)
        {
            //todo: make sure driving at constant heading works
            drive.moveRobotAtConstantHeading(pilotInputCurve.getOuput(xMotionAxis) * tFactor,
                    pilotInputCurve.getOuput(yMotionAxis) * tFactor,
                    pilotInputCurve.getOuput(rotationAxis) * rFactor,
                    targetHeading);
        }
        else
        {
            targetHeading = getAngularOrientationWithOffset();

            drive.moveRobot(pilotInputCurve.getOuput(xMotionAxis) * tFactor,
                    pilotInputCurve.getOuput(yMotionAxis) * tFactor,
                    pilotInputCurve.getOuput(rotationAxis) * rFactor);
        }

        telemetry.addData("eTime: ", eTime);
    }
}
