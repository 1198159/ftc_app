package org.firstinspires.ftc.team6220;

/*
    Contains methods for accepting and interpreting pilot and co-pilot input
*/
abstract public class MasterTeleOp extends MasterOpMode
{

    FIRFilter pilotInputFilter;
    //                                                   y = -1/8x^3 + 0x^2 + 1/2x + 0
    Polynomial pilotInputCurve = new Polynomial(new double[]{ 0.0, 0.5, 0.0, -0.125 });

    void driveRobotWithJoysticks(double xMotionAxis, double yMotionAxis, double rotationAxis)
    {
        drive.moveRobot(pilotInputCurve.getOuput(xMotionAxis),
                        pilotInputCurve.getOuput(yMotionAxis),
                        pilotInputCurve.getOuput(rotationAxis) );
    }
}
