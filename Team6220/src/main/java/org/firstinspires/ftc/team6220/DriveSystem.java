package org.firstinspires.ftc.team6220;

/*
    Contains the robot's drive assemblies, and gives them power or velocity targets based upon robot velocity, power, or position targets.

    INPUTS:
      -Target powers, velocities, or positions (e.g. a driver input)
      -Positions or velocities from sensors (e.g. the vuforia position)
      -Sensor errors (e.g. +/- 1cm)

    OUTPUTS:
      -Estimated robot location using combined sensor inputs
*/
public class DriveSystem
{
    private DriveAssembly[] assemblies;
    private PIDFilter PIDControlFilter;
    public PIDEnforcementMode enforcementMode;
    public Transform2D robotLocation;
    //TODO add Kalman filter for position estimation

    public DriveSystem(DriveAssembly[] driveAssemblyArray, Transform2D initialLocation, PIDFilter filter, PIDEnforcementMode mode)
    {
        this.assemblies = driveAssemblyArray;
        this.robotLocation = initialLocation;
        this.PIDControlFilter = filter;
        this.enforcementMode = mode;
    }

    //TODO add updateRobotLocation() and getRobotLocation()


    //TODO make this more configurable
    //moves the robot according to the PIDEnforcement mode with three inputs
    //  NONE            -  Uses motor powers and no pid control
    //  DERIVATIVE      -  Tries to keep the robot at the input velocities
    //  HIGHEST_ORDER   -  Tries to move the robot in a straight line to the location from its current location (REQUIRES THAT ROBOT LOCATION)
    public void moveRobot(double x, double y, double w)
    {
        double[] rawPowers = new double[]{0.0,0.0,0.0,0.0};

        //TODO see beginning of MasterOpMode
        //calculate motor powers proportionally
        for (int corner = 0; corner < 4; corner++)
        {
            rawPowers[corner] =
                    w
                    + assemblies[corner].location.y * x
                    + assemblies[corner].location.x * y
                    ;
        }
        //scales values such that they will remain in proportion in the case that they would "overflow", e.g. [0.4,0.6,1.0,2.0] becomes [0.2,0.3,0.5,1.0]
        double scalingFactor = 1.0;
        double largest = SequenceUtilities.getLargestMagnitude(rawPowers);
        if (largest > 1.0)
        {
            scalingFactor = largest;
        }

        //write motor powers
        for (int corner = 0; corner < 4; corner++)
        {
            assemblies[corner].setPower(rawPowers[corner]/scalingFactor);
        }

    }

}
