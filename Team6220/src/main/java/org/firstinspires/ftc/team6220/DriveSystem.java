package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
    Contains the robot's drive assemblies, and gives them power or velocity targets based upon robot velocity, power, or position targets.

    INPUTS:
      -Target powers, velocities, or positions (e.g. a driver input)
      -Positions or velocities from sensors (e.g. the vuforia position)
      -Sensor errors (e.g. +/- 1cm)

    OUTPUTS:
      -Estimated robot location using combined sensor inputs
*/

public class DriveSystem implements ConcurrentOperation
{
    private DriveAssembly[] assemblies;
    private PIDFilter[] LocationControlFilter = new PIDFilter[2];
    private PIDFilter RotationControlFilter;
    public Transform2D robotLocation;
    MasterOpMode currentOpmode; //provide access to current opmode so we can use telemetry commands
    VuforiaHelper vuforiaHelper;
    //TODO add Kalman filter for position estimation

    public DriveSystem(MasterOpMode opmode, VuforiaHelper vuforia, DriveAssembly[] driveAssemblyArray, Transform2D initialLocation, PIDFilter[] filter)
    {
        this.currentOpmode = opmode;
        this.vuforiaHelper = vuforia;
        this.assemblies = driveAssemblyArray;
        this.robotLocation = initialLocation;
        this.LocationControlFilter[0] = filter[0];
        this.LocationControlFilter[1] = filter[1];
        this.RotationControlFilter = filter[2];
    }

    @Override
    public void initialize(HardwareMap hMap)
    {

    }

    @Override
    public void update(double eTime)
    {

    }


    //TODO add updateRobotLocation() and getRobotLocation()
    //PID-driven navigation to a point
    //TODO add ability to use non "zig-zag" paths
    //call once per loop
    //assumes robot position had already been updated
    public double[] navigateTo(Transform2D target)
    {
        /*
        float[] l = vuforiaHelper.getRobotLocation();
        l[0] = l[0]/1000;
        l[1] = l[1]/1000;
        //update location
        robotLocation.SetPositionFromFloatArray(l);
        */
        //update error terms
        LocationControlFilter[0].roll(target.x - robotLocation.x);
        LocationControlFilter[1].roll(target.y - robotLocation.y);
        RotationControlFilter.roll(currentOpmode.normalizeRotationTarget(target.rot, robotLocation.rot));
        double xRate = LocationControlFilter[0].getFilteredValue();
        double yRate = LocationControlFilter[1].getFilteredValue();
        double wRate = target.rot - robotLocation.rot;       //RotationControlFilter.getFilteredValue();

        //makes sure that the robot does not move too slowly when nearing its target
        if(Math.abs(xRate) > 1)
        {
            xRate = Math.signum(xRate);
        }
        if(Math.abs(yRate) > 1)
        {
            yRate = Math.signum(yRate);
        }
        if(Math.abs(wRate) > 1)
        {
            wRate = Math.signum(wRate);
        }
        else if(Math.abs(wRate) < 0.3)
        {
            wRate = 0.3 * Math.signum(wRate);
        }


        writeToMotors(getMotorPowersFromMotion(new Transform2D(xRate, yRate, wRate)));
        return new double[]{xRate,yRate,wRate};
    }

    //TODO make assemblies.location represent their actual values
    //return an array with the motor powers for the requested motion
    //                           horizontal vertical  rotation
    public double[] getMotorPowersFromMotion(Transform2D requestedMotion)
    {
        double[] rawPowers = new double[]{0.0,0.0,0.0,0.0};

        //TODO see beginning of MasterOpMode
        //calculate motor powers proportionally
        for (int corner = 0; corner < 4; corner++)
        {
            rawPowers[corner] =
                    requestedMotion.rot
                    + Math.signum(assemblies[corner].location.y) * requestedMotion.x         //assemblies[corner].location.y works as sine of the angle of each motor
                    + Math.signum(assemblies[corner].location.x) * requestedMotion.y         //assemblies[corner].location.x works as cosine of the angle of each motor
                    ;
        }
        //scales values so that they will remain in proportion in the case that they would "overflow"; e.g. [0.4,0.6,1.0,2.0] becomes [0.2,0.3,0.5,1.0]
        double scalingFactor = 1.0;
        double largest = SequenceUtilities.getLargestMagnitude(rawPowers);
        if (largest > 1.0)
        {
            scalingFactor = largest;
        }

        return SequenceUtilities.scalarMultiply(rawPowers,1/scalingFactor);

    }

    //give the drive a command
    public void writeToMotors(double[] powers)
    {
        //write motor powers
        for (int corner = 0; corner < 4; corner++)
        {
            double power = powers[corner];
            assemblies[corner].setPower(power);
            //currentOpmode.telemetry.addData( corner + ": ", power);
        }
    }

    //generate motor powers and write
    public void moveRobot(double x, double y, double w)
    {
        writeToMotors(getMotorPowersFromMotion(new Transform2D(x,y,w)));
    }

    //estimate the robot's last motion using encoders
    //returns an average, so more frequent calls will be noisier but will miss some events
    //note that this is a rate of change in deg/s and m/s, not a position and orientation difference
    public Transform2D getRobotMotionFromEncoders(double eTime)
    {
        Transform2D diff = new Transform2D(0,0,0);
        double[] motion = new double[]{0,0};
        for (int corner = 0; corner < 4; corner++)
        {
            diff.rot += assemblies[corner].getEncoderRadialDifferential(eTime);
        }
        for(int corner = 0; corner < 4; corner++)
        {
            motion = SequenceUtilities.vectorAdd(motion,assemblies[corner].getEncoderVectorDerivative(eTime));
        }
        diff.x = motion[0];
        diff.y = motion[1];
        return diff;
    }

}
