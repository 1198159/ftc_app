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
    double currentAngle = 0.0;

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

        vuforiaHelper = new VuforiaHelper();

        //TODO decide if we should initialize at opmode level
        //                      drive assemblies   initial loc:     x    y    w
        drive = new DriveSystem( this, vuforiaHelper, driveAssemblies,  new Transform2D(0.0, 0.0, 0.0),
                new PIDFilter[]{
                        new PIDFilter(0.8,0.0,0.0),    //x location control
                        new PIDFilter(0.8,0.0,0.0),    //y location control
                        new PIDFilter(1/1500,0.0,0.0)} ); //rotation control

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

    //TODO test encoder function; likely has errors
    //keeps track of the robot's location on the field based on Encoders and IMU; make sure to call once each loop
    public Transform2D updateLocationUsingEncoders()
    {
        //stands for elapsed time
        double eTime;

        //get time when loop starts
        double startTime = System.nanoTime()/1000/1000/1000;
        double finalTime = 0;

        eTime = finalTime - startTime;

        //x and y positions not considering robot rotation
        double xRawPosition = 0.0;
        double yRawPosition = 0.0;

        //x and y positions taking into account robot rotation
        double xLocation;
        double yLocation;

        //angles relative to starting angle used to determine our x and y positions
        double xAngle;
        double yAngle;

        Transform2D location;

        //converted to radians for Math.sin() function
        currentAngle = imu.getAngularOrientation().firstAngle * Math.PI / 180;

        //angle in degrees for return value
        double currentAngleDegrees = imu.getAngularOrientation().firstAngle;

        EncoderFR = driveAssemblies[FRONT_RIGHT].motor.getCurrentPosition();
        EncoderFL = driveAssemblies[FRONT_LEFT].motor.getCurrentPosition();
        EncoderBL = driveAssemblies[BACK_LEFT].motor.getCurrentPosition();
        EncoderBR = driveAssemblies[BACK_RIGHT].motor.getCurrentPosition();

        //math to calculate x and y positions based on encoder ticks and robot angle
        //factors after EncoderFL are equivalent to circumference / encoder ticks per rotation
        //robotXPos = Math.cos(currentAngle) * (EncoderFL + EncoderFR) * 2 * Math.PI * 0.1016 / (1120 * Math.pow(2, 0.5));
        //robotYPos = Math.sin(currentAngle) * (EncoderFL - EncoderFR) * 2 * Math.PI * 0.1016 / (1120 * Math.pow(2, 0.5));

        //not currently in use
        Transform2D motion = drive.getRobotMotionFromEncoders();

        //these are shorthand for the encoder derivative for each motor and will be plugged into our encoder function
        double FLencDerivative = driveAssemblies[BACK_RIGHT].getLinearEncoderDerivative();
        double FRencDerivative = driveAssemblies[BACK_RIGHT].getLinearEncoderDerivative();
        double BLencDerivative = driveAssemblies[BACK_RIGHT].getLinearEncoderDerivative();
        double BRencDerivative = driveAssemblies[BACK_RIGHT].getLinearEncoderDerivative();


        yAngle = Math.PI / 2 + currentAngle;
        xAngle = 90 - yAngle;

        //math to calculate x and y positions based on encoder ticks and not accounting for angle
        xRawPosition += eTime * ((FLencDerivative + FRencDerivative -(BLencDerivative + BRencDerivative)) / (4 * Math.sqrt(2)));
        yRawPosition += eTime * ((FLencDerivative + BLencDerivative -(FRencDerivative + BRencDerivative)) / (4 * Math.sqrt(2)));

        //final location utilizing angles
        xLocation = yRawPosition * Math.cos(yAngle) + xRawPosition * Math.cos(xAngle);
        yLocation = yRawPosition * Math.sin(yAngle) - xRawPosition * Math.sin(xAngle);

        location = new Transform2D(xLocation, yLocation, currentAngleDegrees);

        telemetry.addData("X:", xLocation);
        telemetry.addData("Y:", yLocation);
        telemetry.addData("W:", currentAngleDegrees);
        telemetry.update();

        //get time at end of loop
        finalTime = System.nanoTime()/1000/1000/1000;

        return location;
    }

    public void navigateUsingEncoders(Transform2D target)
    {
        double xTolerance = .010;
        double yTolerance = .010;
        double wTolerance = 3.0;

        Transform2D newLocation = updateLocationUsingEncoders();

        while ((Math.abs(target.x - drive.robotLocation.x) > xTolerance) || (Math.abs(target.y - drive.robotLocation.y) > yTolerance)|| (Math.abs(target.rot - drive.robotLocation.rot) > wTolerance))
        {
            drive.navigateTo(target);

            newLocation = updateLocationUsingEncoders();
        }
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

    public void stopAllDriveMotors()
    {
        driveAssemblies[FRONT_RIGHT].setPower(0.0);
        driveAssemblies[FRONT_LEFT].setPower(0.0);
        driveAssemblies[BACK_LEFT].setPower(0.0);
        driveAssemblies[BACK_RIGHT].setPower(0.0);
    }
}
