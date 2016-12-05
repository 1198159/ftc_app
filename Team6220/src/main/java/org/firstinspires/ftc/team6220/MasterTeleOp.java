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
    FIRFilter pilotInputFilter = new FIRFilter(new Polynomial(new double[]{100,-1,0.0025}),200);

    //                                                   y = 1/2x^3 + 0x^2 + 1/2x + 0
    Polynomial pilotInputCurve = new Polynomial(new double[]{ 0.0, 0.5, 0.0, 0.5 });

    void driveRobotWithJoysticks(double xMotionAxis, double yMotionAxis, double rotationAxis, boolean slow)
    {
        double factor = 1.0;
        if(slow)
        {
            factor = 0.4;
        }
        drive.moveRobot(pilotInputCurve.getOuput(xMotionAxis)*factor,
                        pilotInputCurve.getOuput(yMotionAxis)*factor,
                        pilotInputCurve.getOuput(rotationAxis)*factor);
    }
}
