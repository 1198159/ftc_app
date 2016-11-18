package org.firstinspires.ftc.team6220;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;



/*

*/

abstract public class MasterOpMode extends LinearOpMode
{
    //Java Enums cannot act as integer indices as they do in other languages
    //To maintain compatabliity with with a FOR loop, we
    //TODO find if java supports an order-independent loop for each member of an array
    //e.g. in python that would be:   for assembly in driveAssemblies:
    //                                    assembly.doStuff()
    public final int FRONT_RIGHT = 0;
    public final int FRONT_LEFT  = 1;
    public final int BACK_LEFT   = 2;
    public final int BACK_RIGHT  = 3;

    DcMotor CollectorMotor;

    private int EncoderFR = 0;
    private int EncoderFL = 0;
    private int EncoderBL = 0;
    private int EncoderBR = 0;

    double robotXPos = 0;
    double robotYPos = 0;

    //TODO deal with angles at all starting positions
    double currentAngle = 0;

    BNO055IMU imu;

    DriveAssembly[] driveAssemblies;

    DriveSystem drive;

    VuforiaHelper vuforiaHelper;

    MotorToggler motorToggler;
    MotorToggler motorTogglerReverse;

    public void initializeHardware()
    {
        //create a driveAssembly array to allow for easy access to motors
        driveAssemblies = new DriveAssembly[5];

        //TODO adjust correction factor if necessary
        //TODO fix all switched front and back labels on motors
        //our robot uses an omni drive, so our motors are positioned at 45 degree angles to motor positions on a normal drive.
                                                                        //mtr,                                       x,   y,   rot,  gear, radius, correction factor
        driveAssemblies[FRONT_RIGHT] = new DriveAssembly(hardwareMap.dcMotor.get("motorBackRight"),new Transform2D( 1.0, 1.0, 135), 1.0, 0.1016, 1.0);
        driveAssemblies[FRONT_LEFT]  = new DriveAssembly(hardwareMap.dcMotor.get("motorBackLeft") ,new Transform2D(-1.0, 1.0, 225), 1.0, 0.1016, 1.0);
        driveAssemblies[BACK_LEFT]   = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontLeft")  ,new Transform2D(-1.0,-1.0, 315), 1.0, 0.1016, 1.0);
        driveAssemblies[BACK_RIGHT]  = new DriveAssembly(hardwareMap.dcMotor.get("motorFrontRight") ,new Transform2D( 1.0,-1.0,  45), 1.0, 0.1016, 1.0);

        CollectorMotor = hardwareMap.dcMotor.get("motorCollector");

        //TODO tune our own drive PID loop using DriveAssemblyPID instead of build-in P/step filter
        //TODO Must be disabled if motor encoders are not correctly reporting
        driveAssemblies[FRONT_RIGHT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[FRONT_LEFT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[BACK_LEFT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveAssemblies[BACK_RIGHT].motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CollectorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //TODO decide if we should initialize at opmode level
        //                      drive assemblies   initial loc:     x    y    w
        drive = new DriveSystem( driveAssemblies,  new Transform2D(0.0, 0.0, 0.0),
                new PIDFilter[]{
                        new PIDFilter(2.0,0.0,0.0),    //x location control
                        new PIDFilter(2.0,0.0,0.0),    //y location control
                        new PIDFilter(0.017,0.0,0.0)} ); //rotation control

        motorToggler = new MotorToggler(CollectorMotor, 1.0);
        motorTogglerReverse = new MotorToggler(CollectorMotor, -1.0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //TODO check encoder and imu location function
    //keeps track of the robot's location on the field based on Encoders and IMU
    public Transform2D updateLocationUsingEncoders()
    {
        Transform2D location;

        currentAngle = imu.getAngularOrientation().firstAngle;

        //CodeReview: Bug? This is either a bug or redundant: all are pointing to the same array element FRONT_RIGHT
        EncoderFR = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderFL = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderBL = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderBR = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();

        //math to calculate x and y positions based on encoder ticks and robot angle
        //factors after EncoderFL are equivalent to circumference / encoder ticks per rotation
        robotXPos = Math.cos(currentAngle) * (EncoderFR + EncoderFL) * 2 * Math.PI * 0.1016 / 1120 / Math.pow(2, 0.5);
        robotYPos = Math.sin(currentAngle) * (EncoderFR + EncoderFL) * 2 * Math.PI * 0.1016 / 1120 / Math.pow(2, 0.5);

        location = new Transform2D(robotXPos, robotYPos, currentAngle);

        telemetry.addData("X:", robotXPos);
        telemetry.addData("Y:", robotYPos);
        telemetry.update();

        return location;
    }

    public void navigateUsingEncoders(Transform2D target)
    {
        updateLocationUsingEncoders();
        drive.navigateTo(target);
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
}
