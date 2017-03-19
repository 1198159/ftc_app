package org.firstinspires.ftc.team6220;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.util.Timer;

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
    //made public for use in turnTo
    public PIDFilter RotationControlFilter;
    private PIDFilter DriveStraightFilter;
    public Transform2D robotLocation;
    MasterOpMode currentOpMode; //provide access to current opmode so we can use telemetry commands
    VuforiaHelper vuforiaHelper;
    //TODO add Kalman filter for position estimation

    public DriveSystem(MasterOpMode opmode, VuforiaHelper vuforia, DriveAssembly[] driveAssemblyArray, Transform2D initialLocation, PIDFilter[] filter)
    {
        this.currentOpMode = opmode;
        this.vuforiaHelper = vuforia;
        this.assemblies = driveAssemblyArray;
        this.robotLocation = initialLocation;
        this.LocationControlFilter[0] = filter[0];
        this.LocationControlFilter[1] = filter[1];
        this.RotationControlFilter = filter[2];
        this.DriveStraightFilter = filter[3];
    }

    @Override
    public void initialize(HardwareMap hMap)
    {

    }

    @Override
    public void update(double eTime)
    {

    }

    //TODO add ability to use non "zig-zag" paths
    //PID-driven navigation to a point; call once per loop
    //IMPORTANT:  assumes robot position has already been updated
    public double[] navigateTo(double targetX, double targetY)
    {
        //updates the error terms
        LocationControlFilter[0].roll(targetX - robotLocation.x);
        LocationControlFilter[1].roll(targetY - robotLocation.y);
        double xRate = 0.3 * LocationControlFilter[0].getFilteredValue();
        double yRate = 0.3 * LocationControlFilter[1].getFilteredValue();

        //makes sure that none of our rates are too large
        if(Math.abs(xRate) > 0.3)
        {
            xRate = 0.3 * Math.signum(xRate);
        }

        if(Math.abs(yRate) > 0.3)
        {
            yRate = 0.3 * Math.signum(yRate);
        }

        //Local and global orientation do not always match up.  For instance, if the robot is rotated
        //90 degrees, local x motion will result in global y motion.
        //This calculation corresponds to the new coordinates of a point rotated to an angle.  The 90
        //subtracted from beaconActivationAngle has to do with how our robot's local orientation
        //was defined
        double localXRate = xRate * Math.cos(currentOpMode.beaconActivationAngle - 90) - yRate * Math.sin(currentOpMode.beaconActivationAngle - 90);
        double localYRate = -xRate * Math.sin(currentOpMode.beaconActivationAngle - 90) - yRate * Math.cos(currentOpMode.beaconActivationAngle - 90);

        writeToMotors(getMotorPowersFromMotion(new Transform2D(localXRate, localYRate, 0.0)));

        return new double[]{localXRate, localYRate};
    }

    //@todo navigating in the y direction does not work
    //used to navigate along a single axis instead of a zig-zag path
    //PID-driven navigation to a point; call once per loop
    //IMPORTANT:  assumes robot position has already been updated
    public void NavigateAxially(boolean redSide, boolean x, double targetPosition, double targetAngle)
    {
        double posRate;

        //updates the error terms
        if (redSide)
        {
            LocationControlFilter[0].roll(-(targetPosition - robotLocation.x));
            posRate = 0.3 * LocationControlFilter[0].getFilteredValue();
        }
        else
        {
            LocationControlFilter[1].roll(targetPosition - robotLocation.y);
            posRate = 0.3 * LocationControlFilter[1].getFilteredValue();
        }
        double angleDiff = currentOpMode.normalizeRotationTarget(targetAngle, robotLocation.rot);
        RotationControlFilter.roll(angleDiff);
        double wRate = 0.3 * RotationControlFilter.getFilteredValue();
        currentOpMode.telemetry.addData("Angle diff: ", angleDiff);

        //makes sure that none of our rates are too large
        if(Math.abs(posRate) > 0.3)
        {
            posRate = 0.3 * Math.signum(posRate);
        }

        if(Math.abs(wRate) > 0.3)
        {
            wRate = 0.3 * Math.signum(wRate);
        }

        //changes axis of motion based on input
        if (x)
        {
            writeToMotors(getMotorPowersFromMotion(new Transform2D(posRate, 0.0, 0.0 * wRate)));
        }
        else
        {
            writeToMotors(getMotorPowersFromMotion(new Transform2D(0.0, -posRate, 0.0 * wRate)));
        }

        //return this.getMotorPowersFromMotion(new Transform2D(posRate, 0.0 , wRate));
    }

    //TODO make assemblies.location represent their actual values
    //returns an array with the motor powers for the requested motion
    //                                      horizontal vertical  rotation
    public double[] getMotorPowersFromMotion(Transform2D requestedMotion)
    {
        double[] rawPowers = new double[]{0.0,0.0,0.0,0.0};

        //TODO see beginning of MasterOpMode
        //calculates motor powers proportionally
        for (int corner = 0; corner < 4; corner++)
        {
            rawPowers[corner] =
                    requestedMotion.rot
                    + Math.signum(assemblies[corner].location.y) * requestedMotion.x         //assemblies[corner].location.y works as sine of the angle of each motor
                    + Math.signum(assemblies[corner].location.x) * requestedMotion.y         //assemblies[corner].location.x works as cosine of the angle of each motor
                    ;
            //currentOpMode.telemetry.addData("etime: ", )
        }
        //scales values so that they will remain in proportion in the case that they would "overflow"; e.g. [0.4,0.6,1.0,2.0] becomes [0.2,0.3,0.5,1.0]
        double scalingFactor = 1.0;
        double largest = SequenceUtilities.getLargestMagnitude(rawPowers);
        if (largest > 1.0)
        {
            scalingFactor = largest;
        }

        return SequenceUtilities.scalarMultiply(rawPowers, 1/scalingFactor);
    }

    //todo: code duplicate; incorporate both getMotorPowers into a single function accounting for heading
    //ensures robot will drive straight while moving
    //this assumes that requestedMotion.rot is ignored; it is calculated later
    public double[] getMotorPowersAccountingForHeading(Transform2D requestedMotion, double targetOrientation)
    {
        double[] rawPowers = new double[]{0.0,0.0,0.0,0.0};

        double currentAngle = currentOpMode.getAngularOrientationWithOffset();

        requestedMotion.rot = currentOpMode.normalizeRotationTarget(targetOrientation, currentAngle) / 180.0;

        /*
        //ensures that the robot is driving at the correct heading
        DriveStraightFilter.roll(currentOpMode.normalizeRotationTarget(targetOrientation, currentAngle));
        requestedMotion.rot =  DriveStraightFilter.getFilteredValue();
        */

        if (Math.abs(requestedMotion.rot) > 1.0)
        {
            requestedMotion.rot = Math.signum(requestedMotion.rot);
        }

        //requestedMotion.rot = 0;

        //data necessary for debugging
        currentOpMode.telemetry.addData("Rotation: ", requestedMotion.rot);
        currentOpMode.telemetry.addData("headingDiff: ", currentOpMode.normalizeRotationTarget(targetOrientation, currentAngle));

        //todo: see beginning of MasterOpMode
        //calculate motor powers proportionally
        for (int corner = 0; corner < 4; corner++)
        {
            rawPowers[corner] =
                    //todo: check to see if sign on requestedMotion.rot is correct
                            -requestedMotion.rot +
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

        return SequenceUtilities.scalarMultiply(rawPowers, 1 / scalingFactor);
    }

    //gives the drive motors a command
    public void writeToMotors(double[] powers)
    {
        //write motor powers
        for (int corner = 0; corner < 4; corner++)
        {
            double power = powers[corner];
            //power = 0.25; // test value for debugging
            assemblies[corner].setPower(power);

            currentOpMode.telemetry.addData( corner + ": ", power);
        }
    }

    //generates motor powers and writes values to them
    public void moveRobot(double x, double y, double w)
    {
        //allows driver to pick which end of the robot acts as the front
        if(!currentOpMode.leftButtonPusherAsFront)
        {
            writeToMotors(getMotorPowersFromMotion(new Transform2D(x, y, w)));
        }
        else
        {
            writeToMotors(getMotorPowersFromMotion(new Transform2D(y, -x, w)));
        }
    }

    //TODO see above
    //generate motor powers and write values to them; drives at an ideal angle
    public void moveRobotAtConstantHeading(double x, double y, double w, double targetHeading)
    {
        writeToMotors(getMotorPowersAccountingForHeading(new Transform2D(x, y, w), targetHeading));
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
