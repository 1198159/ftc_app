package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.List;



/*

*/

abstract public class MasterOpMode extends LinearOpMode
{
    //TODO: deal with angles at all starting positions
    double currentAngle = 0.0;

    //used to create global coordinates by adjusting the imu heading based on the robot's starting orientation
    private double headingOffset = 0.0;

    //used to ensure that the robot drives straight when not attempting to turn
    double targetHeading = 0.0 + headingOffset;

    VuforiaHelper vuforiaHelper;

    ElapsedTime timer = new ElapsedTime();
    double lTime = 0;

    DriverInput driver1;
    DriverInput driver2;

    BNO055IMU imu;

    List<ConcurrentOperation> callback = new ArrayList<>();
    //currently not in use
    /*
    MotorToggler motorToggler;
    MotorToggler motorTogglerReverse;
    */

    public void initializeHardware()
    {
        driver1 = new DriverInput(gamepad1);
        driver2 = new DriverInput(gamepad2);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        //CodeReview: Did we include the calibration json somewhere so it can be found in our program?
        //            If we are going to reference this file, it has to exist, and has to be where the
        //            calibration sample opmode puts it (so it can be found)
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        callback.add(driver1);
        callback.add(driver2);

        for(ConcurrentOperation item : callback)
        {
            item.initialize(hardwareMap);
        }
    }

    public void updateCallback(double eTime)
    {
        for(ConcurrentOperation item : callback)
        {
            item.update(eTime);
        }
    }

    //other opmodes must go through this method to prevent others from blithely changing headingOffset
    public void setRobotStartingOrientation(double newValue)
    {
        headingOffset = newValue;
    }

    //prevents angle differences from being out of range
    public double normalizeRotationTarget(double finalAngle, double initialAngle)
    {
        double diff = finalAngle - initialAngle;

        while (Math.abs(diff) > 180)
        {
            diff -= Math.signum(diff) * 360;
        }

        return diff;
    }

    //prevents a single angle from being outside the range -180 to 180 degrees
    public double normalizeAngle(double rawAngle)
    {
        while (Math.abs(rawAngle) > 180)
        {
            rawAngle -= Math.signum(rawAngle) * 360;
        }

        return rawAngle;
    }

    //takes into account headingOffset to utilize global orientation
    public double getAngularOrientationWithOffset()
    {
        double correctedHeading = normalizeAngle(imu.getAngularOrientation().firstAngle + headingOffset);

        return correctedHeading;
    }

    //uses vuforia instead of imu
    public double getRobotAngleUsingVuforia()
    {
        vuforiaHelper.updateLocation();
        return Orientation.getOrientation(vuforiaHelper.lastKnownLocation, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle + headingOffset;
    }

    //wait a number of milliseconds
    public void pause(int t) throws InterruptedException
    {
        //we don't use System.currentTimeMillis() because it can be inconsistent
        long initialTime = System.nanoTime();
        while((System.nanoTime() - initialTime)/1000/1000 < t)
        {
            idle();
        }
    }

    //todo add motors when drive system is decided
    public void stopAllDriveMotors()
    {

    }
}
