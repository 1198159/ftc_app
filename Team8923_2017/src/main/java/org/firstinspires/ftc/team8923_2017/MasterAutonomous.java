package org.firstinspires.ftc.team8923_2017;

/*
 * Holds all code necessary to run the robot in autonomous controlled mode
 */

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.hardware.Camera;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;
public abstract class MasterAutonomous extends Master
{


    private ElapsedTime runtime = new ElapsedTime();



    enum Alliance
    {
        RED(),
        BLUE();
    }

    enum StartPositions
    {
        LEFT(0),
        RIGHT(0);

        /*
        RED_LEFT(),
        RED_RIGHT(),
        BLUE_LEFT(),
        BLUE_RIGHT();
        */

        public final double val;
        StartPositions(double i)
        {
            val = i;
        }
    }

    static final double COUNTS_PER_MOTOR_REV = 1120;
    static final double DRIVE_GEAR_REDUCTION = 1.0; // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0; // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double COUNTS_PER_MM = COUNTS_PER_INCH / 25.4;

    // These positions are the allow the JJ to gradually lower
    double SERVO_JJ_MIDDLE1 = 0.5;
    double SERVO_JJ_MIDDLE2 = 0.4;
    double SERVO_JJ_MIDDLE3 = 0.35;
    double SERVO_JJ_MIDDLE4 = 0.3;
    double SERVO_JJ_MIDDLE5 = 0.23;

    ElapsedTime GGLiftTimer = new ElapsedTime();
    boolean liftMoving = false;

    Alliance alliance = Alliance.RED;
    StartPositions startPosition = StartPositions.LEFT;
    boolean setupFinished = false;
    int delayTime = 0;

    double robotX;
    double robotY;
    double robotAngle;
    double kMove = 1/600.0;


    // Used to calculate distance traveled between loops
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    double headingOffset = 0.0;

    private static final double MAX_DRIVE_POWER = 1.0;
    private static final double MIN_DRIVE_POWER = 0.15;
    private static final double TURN_POWER_CONSTANT = 1.0 / 175.0;
    private static final double DRIVE_POWER_CONSTANT = 1.0 / 1750.0;

    VuforiaLocalizer.CloseableFrame frame;
    Image image = null;
    Image imageRGB565 = null;
    int imageFormat;
    Bitmap bm; // android.graphics

    VuforiaTrackables relicTrackables;
    VuforiaTrackable relicTemplate;
    public RelicRecoveryVuMark vuMark;

    public float leftColorHSV[] = {0f, 0f, 0f};
    public float rightColorHSV[] = {0f, 0f, 0f};

    //Matrix34F rawPose;
    int color = 0;
    float[] HsvSum = {0,0,0};
    float[] HSV = {0,0,0};
    float[] HsvOut = {0,0,0};

    int numRedPixels = 0;
    int numBluePixels = 0;
    int numOtherPixels = 0;

    boolean isVuMarkVisible;
    boolean isLeftJewelRed;

    VuforiaLocalizer vuforia;

    OpenGLMatrix pose;


    //Color Sensors
    ColorSensor sensorTopLeft;
    ColorSensor sensorTopRight;
    ColorSensor sensorBottomRight;
    //DeviceInterfaceModule cdim;

    // we assume that the LED pin of the RGB sensor is connected to
    // digital port 5 (zero indexed).

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValuesTopLeft[] = {0F,0F,0F};
    float hsvValuesTopRight[] = {0F,0F,0F};
    float hsvValuesBottomRight[] = {0F,0F,0F};

    // get a reference to the RelativeLayout so we can change the background
    // color of the Robot Controller app to match the hue detected by the RGB sensor.
    //int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

    // bPrevState and bCurrState represent the previous and current state of the button.
    boolean bPrevState = false;
    boolean bCurrState = false;

    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    // values is a reference to the hsvValues array.
    //final float values[] = hsvValues;
    //static final int LED_CHANNEL = 5;

/*
    void ChooseOptions()
    {
        while (!setupFinished)
        {
            if (gamepad1.x)
                alliance = Alliance.BLUE;
            else if (gamepad1.b)
                alliance = Alliance.RED;

            if (gamepad1.dpad_left)
                startPosition = StartPositions.LEFT;
            else if (gamepad1.dpad_right)
                startPosition = StartPositions.RIGHT;

            if(gamepad1.dpad_up)
                delayTime++;
            else if (gamepad1.dpad_down && delayTime > 0)
                delayTime --;

            if(gamepad1.start)
                setupFinished = true;

            telemetry.addData("Alliance (Blue/Red): (X/B)", alliance.name());
            telemetry.addData("Start Position (Left/Right): (Dpad Left/Dpad Right)", startPosition.name());
            telemetry.addData("Delay Time (+/-): (Dpad Up/Dpad Down)", delayTime);
            telemetry.update();

            while (!buttonsAreReleased(gamepad1))
                idle();
        }
    }
*/
    void InitAuto()
    {
        InitHardwareAutonomous();
/*
        switch (alliance)
        {
            case RED:
                robotX = 3048;
                switch (startPosition)
                {
                    case LEFT:
                        robotY = 700;
                        break;
                    case RIGHT:
                        robotY = 2470;
                        break;
                }
                break;

            case BLUE:
                robotX = 610;
                switch (startPosition)
                {
                    case LEFT:
                        robotY = 2470;
                        break;
                    case RIGHT:
                        robotY = 700;
                        break;
                }
                break;
        }
*/
        robotAngle = 90;
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;
    }


    void InitHardwareAutonomous()
    {
        // Motors here
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");
        motorGG = hardwareMap.dcMotor.get("motorGG");

        // Servos here
        servoJJ = hardwareMap.get(Servo.class, "servoJJ");
        servoGGUL = hardwareMap.get(Servo.class, "servoGGUL");
        servoGGUR = hardwareMap.get(Servo.class, "servoGGUR");
        servoGGDL = hardwareMap.get(Servo.class, "servoGGDL");
        servoGGDR = hardwareMap.get(Servo.class, "servoGGDR");

        servoJJ.setPosition(SERVO_JJ_UP);
        //servoGGL.setPosition(0.35);
        //servoGGR.setPosition(0.55);

        // Reset encoders
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Run without encoders
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Use run to position
        motorGG.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Use brake mode
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sensors here
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        sensorTopRight = hardwareMap.get(ColorSensor.class, "sensorTopRight");
        sensorTopLeft = hardwareMap.get(ColorSensor.class, "sensorTopLeft");
        sensorBottomRight = hardwareMap.get(ColorSensor.class, "sensorBottomRight");
        GGZero = motorGG.getCurrentPosition();
    }

    void Run() throws InterruptedException //Generic run method for testing purposes now
    {
        switch (alliance)
        {
            case RED:
                driveToPoint(3048, 1490, 90.0, 0.8);
                break;
            case BLUE:
                driveToPoint(610, 1490, 90.0, 0.8);
        }
    }

    void closeGG()
    {
        // Close GG claws
        servoGGUL.setPosition(GGServoPositions.UPPERLEFTFULLCLOSED.val());
        servoGGUR.setPosition(GGServoPositions.UPPERRIGHTFULLCLOSED.val());
        servoGGDL.setPosition(GGServoPositions.LOWERLEFTFULLCLOSED.val());
        servoGGDR.setPosition(GGServoPositions.LOWERRIGHTFULLCLOSED.val());
    }
    void openGG()
    {
        // Open GG claws
        servoGGUL.setPosition(GGServoPositions.UPPERLEFTFULLOPEN.val());
        servoGGUR.setPosition(GGServoPositions.UPPERRIGHTFULLOPEN.val());
        servoGGDL.setPosition(GGServoPositions.LOWERLEFTFULLOPEN.val());
        servoGGDR.setPosition(GGServoPositions.LOWERRIGHTFULLOPEN.val());
    }

    void MoveIMU(double referenceAngle, double moveMM, double targetAngle, double kAngle, double maxSpeed, double timeout)
    {
        //Sets motor encoder values
        newTargetFL = motorFL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetFR = motorFR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetBL = motorBL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetBR = motorBR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);

        runtime.reset(); // used for timeout
        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            // Get the current motor positions
            currentFL = motorFL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();

            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            moveErrorFL = newTargetFL - currentFL; // Calculate the error (value of difference between target position and current position
            speedFL = Math.abs(kMove * moveErrorFL); // Update speed to be the error times a constant
            speedFL = Range.clip(speedFL, 0.15, maxSpeed); // Update speed to be a minimum of 0.15 amd maxspeed
            speedFL = speedFL * Math.signum(moveErrorFL); // Update speed

            moveErrorFR = newTargetFR - motorFR.getCurrentPosition();
            speedFR = Math.abs(kMove * moveErrorFR);
            speedFR = Range.clip(speedFR, 0.15, maxSpeed);
            speedFR = speedFR * Math.signum(moveErrorFR);

            moveErrorBL = newTargetBL - currentBL;
            speedBL = Math.abs(kMove * moveErrorBL);
            speedBL = Range.clip(speedBL, 0.15, maxSpeed);
            speedBL = speedBL * Math.signum(moveErrorBL);

            moveErrorBR = newTargetBR - currentBR;
            speedBR = Math.abs(kMove * moveErrorBR);
            speedBR = Range.clip(speedBR, 0.15, maxSpeed);
            speedBR = speedBR * Math.signum(moveErrorBR);

            // Pivot adjusts the robot to maintain its heading
            targetAngle = adjustAngles(targetAngle); // Makes it so target angle does not wrap
            angleError = currentRobotAngle - targetAngle; // Error equals robot angle times target
            angleError = adjustAngles(angleError); // Makes it so target angle does not wrap
            pivot = angleError * kAngle; // pivot equals the error times a constant

            // Sets values for motor power
            motorPowerFL = -speedFR + pivot;
            motorPowerFR = speedFR + pivot;
            motorPowerBL = -speedFR + pivot;
            motorPowerBR = speedFR + pivot;

            // Set values for motor powers
            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
            idle();
        }

        while (opModeIsActive() && (runtime.seconds() < timeout) && Math.abs(moveErrorFR) > TOL);
        stopDriving();
    }

    // This method isn't used anymore as it was replaced by MoveIMU
    void move(double referenceAngle, double moveMM, double targetAngle, double kAngle, double maxSpeed, double timeout)
    {
        //Sets motor encoder values
        newTargetFL = motorFL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetFR = motorFR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetBL = motorBL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetBR = motorBR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);

        runtime.reset(); // used for timeout
        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            currentFL = motorFL.getCurrentPosition();
            currentFR = motorFR.getCurrentPosition();
            currentBL = motorBL.getCurrentPosition();
            currentBR = motorBR.getCurrentPosition();

            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            moveErrorFL = newTargetFL + currentFL;
            speedFL = Math.abs(kMove * moveErrorFL);
            speedFL = Range.clip(speedFL, 0.15, maxSpeed);
            speedFL = speedFL * Math.signum(moveErrorFL);

            moveErrorFR = newTargetFR + motorFR.getCurrentPosition();
            speedFR = Math.abs(kMove * moveErrorFR);
            speedFR = Range.clip(speedFR, 0.15, maxSpeed);
            speedFR = speedFR * Math.signum(moveErrorFR);

            moveErrorBL = newTargetBL + currentBL;
            speedBL = Math.abs(kMove * moveErrorBL);
            speedBL = Range.clip(speedBL, 0.15, maxSpeed);
            speedBL = speedBL * Math.signum(moveErrorBL);

            moveErrorBR = newTargetBR + currentBR;
            speedBR = Math.abs(kMove * moveErrorBR);
            speedBR = Range.clip(speedBR, 0.15, maxSpeed);
            speedBR = speedBR * Math.signum(moveErrorBR);

            //Sets values for motor power
            motorPowerFL = -speedFR + pivot;
            motorPowerFR = speedFR + pivot;
            motorPowerBL = -speedFR + pivot;
            motorPowerBR = speedFR + pivot;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
            idle();
        }

        while (opModeIsActive() && (runtime.seconds() < timeout) && Math.abs(moveErrorFR) > TOL);
        stopDriving();
    }

    void MoveIMULeft(double referenceAngle, double moveMM, double targetAngle, double kAngle, double maxSpeed, double timeout)
    {
        currentFL = motorFL.getCurrentPosition();
        currentFR = motorFR.getCurrentPosition();
        currentBL = motorBL.getCurrentPosition();
        currentBR = motorBR.getCurrentPosition();

        //Sets motor encoder values
        newTargetFL = motorFL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetFR = motorFR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetBL = motorBL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetBR = motorBR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);

        runtime.reset(); // used for timeout
        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            moveErrorFL = newTargetFL - currentFL;
            speedFL = Math.abs(kMove * moveErrorFL);
            speedFL = Range.clip(speedFL, 0.15, maxSpeed);
            speedFL = speedFL * Math.signum(moveErrorFL);

            moveErrorFR = newTargetFR - currentFR;
            speedFR = Math.abs(kMove * moveErrorFR);
            speedFR = Range.clip(speedFR, 0.15, maxSpeed);
            speedFR = speedFR * Math.signum(moveErrorFR);

            moveErrorBL = newTargetBL - currentBL;
            speedBL = Math.abs(kMove * moveErrorBL);
            speedBL = Range.clip(speedBL, 0.15, maxSpeed);
            speedBL = speedBL * Math.signum(moveErrorBL);

            moveErrorBR = newTargetBR - currentBR;
            speedBR = Math.abs(kMove * moveErrorBR);
            speedBR = Range.clip(speedBR, 0.15, maxSpeed);
            speedBR = speedBR * Math.signum(moveErrorBR);

            targetAngle = adjustAngles(targetAngle);//Makes it so target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            //Sets values for motor power
            motorPowerFL = speedBR + pivot;
            motorPowerFR = speedBR + pivot;
            motorPowerBL = -speedBR + pivot;
            motorPowerBR = -speedBR + pivot;

            //Sets motor power
            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(moveErrorBL) > TOL));

        //Stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    void MoveIMURight(double referenceAngle, double moveMM, double targetAngle, double kAngle, double maxSpeed, double timeout)
    {
        currentFL = motorFL.getCurrentPosition();
        currentFR = motorFR.getCurrentPosition();
        currentBL = motorBL.getCurrentPosition();
        currentBR = motorBR.getCurrentPosition();

        //Sets motor encoder values
        newTargetFL = motorFL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetFR = motorFR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetBL = motorBL.getCurrentPosition() + (int) (moveMM / MM_PER_TICK);
        newTargetBR = motorBR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);

        runtime.reset(); // used for timeout
        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            moveErrorFL = newTargetFL - currentFL;
            speedFL = Math.abs(kMove * moveErrorFL);
            speedFL = Range.clip(speedFL, 0.15, maxSpeed);
            speedFL = speedFL * Math.signum(moveErrorFL);

            moveErrorFR = newTargetFR - currentFR;
            speedFR = Math.abs(kMove * moveErrorFR);
            speedFR = Range.clip(speedFR, 0.15, maxSpeed);
            speedFR = speedFR * Math.signum(moveErrorFR);

            moveErrorBL = newTargetBL - currentBL;
            speedBL = Math.abs(kMove * moveErrorBL);
            speedBL = Range.clip(speedBL, 0.15, maxSpeed);
            speedBL = speedBL * Math.signum(moveErrorBL);

            moveErrorBR = newTargetBR - currentBR;
            speedBR = Math.abs(kMove * moveErrorBR);
            speedBR = Range.clip(speedBR, 0.15, maxSpeed);
            speedBR = speedBR * Math.signum(moveErrorBR);

            targetAngle = adjustAngles(targetAngle);//Makes it so target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            //Sets values for motor power
            motorPowerFL = maxSpeed;
            motorPowerFR = maxSpeed;
            motorPowerBL = -maxSpeed;
            motorPowerBR = -maxSpeed;

            //Sets motor power
            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout));

        //Stop motors
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    void IMUPivot(double referenceAngle, double targetAngle, double MaxSpeed, double kAngle)
    {
        targetAngle =  referenceAngle + targetAngle;//Adds the current angle to the target
        targetAngle = adjustAngles(targetAngle);
        do {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            targetAngle = adjustAngles(targetAngle);//Makes it so the target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            if (pivot >= 0.0)
            {
                pivot = Range.clip(pivot, 0.15, MaxSpeed);
            }
            else
            {
                pivot = Range.clip(pivot, -MaxSpeed, -0.15);
            }

            motorPowerFL = pivot;
            motorPowerFR = pivot;
            motorPowerBL = pivot;
            motorPowerBR = pivot;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
            idle();
        }
        while (opModeIsActive() && (Math.abs(angleError) > AngleTOL));

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    void pivotFixed(double speed)
    {
        motorFL.setPower(speed);
        motorFR.setPower(speed);
        motorBL.setPower(speed);
        motorBR.setPower(speed);
        sleep(3000);
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    void driveToPoint(double targetX, double targetY, double targetAngle, double maxPower) throws InterruptedException
    {
        UpdateRobotLocation();

        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double DISTANCE_TOLERANCE = 40; // In mm
        double ANGLE_TOLERANCE = 5; // In degrees

        // Run until robot is within tolerable distance and angle
        while(!(distanceToTarget < DISTANCE_TOLERANCE && deltaAngle < ANGLE_TOLERANCE) && opModeIsActive())
        {
            UpdateRobotLocation();

            // In case robot drifts to the side
            double driveAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)) - robotAngle;

            // Decrease power as robot approaches target. Ensure it doesn't exceed power limits
            double drivePower = Range.clip(distanceToTarget * DRIVE_POWER_CONSTANT, MIN_DRIVE_POWER, maxPower);

            // In case the robot turns while driving
            deltaAngle = subtractAngles(targetAngle, robotAngle);
            double turnPower = deltaAngle * TURN_POWER_CONSTANT;

            // Set drive motor powers
            driveOmni45(driveAngle, drivePower, turnPower);

            // Recalculate distance for next check
            distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);

            // Inform drivers of robot location
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("RobotAngle", robotAngle);
            idle();
        }
        stopDriving();
    }

    void stopGG()
    {
        motorGG.setPower(0.0);
        idle();
    }

    void moveGG(int ticks)
    {
        // Method raises or lowers GG
        liftMoving = true;
        GGLiftTimer.reset();
        motorGG.setTargetPosition(motorGG.getCurrentPosition() + ticks);
        motorGG.setPower((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 1000.0));
        idle();
    }

    void DropJJ()
    {
        // Drops JJ slowly
        servoJJ.setPosition(SERVO_JJ_MIDDLE);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_MIDDLE1);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_MIDDLE2);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_MIDDLE3);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_MIDDLE4);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_MIDDLE5);
        sleep(200);
        /*servoJJ.setPosition(SERVO_JJ_MIDDLE6);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_DOWN);
        */
    }

    void RetrieveJJ()
    {
        // Raise JJ
        servoJJ.setPosition(SERVO_JJ_UP);
    }

    void UpdateRobotLocation()
    {
        // Update robot angle
        robotAngle = imu.getAngularOrientation().firstAngle - headingOffset;

        // Calculate how far each motor has turned since last time
        int deltaFL = motorFL.getCurrentPosition() - lastEncoderFL;
        int deltaFR = motorFR.getCurrentPosition() - lastEncoderFR;
        int deltaBL = motorBL.getCurrentPosition() - lastEncoderBL;
        int deltaBR = motorBR.getCurrentPosition() - lastEncoderBR;

        // Take average of encoder ticks to find translational x and y components. FR and BL are
        // negative because of the direction at which they turn when going sideways
        double deltaX = (-deltaFL - deltaFR + deltaBL + deltaBR) / 4;
        double deltaY = (-deltaFL + deltaFR - deltaBL + deltaBR) / 4;

        /*
         * Delta x and y are intrinsic to the robot, so they need to be converted to extrinsic.
         * Each intrinsic component has 2 extrinsic components, which are added to find the
         * total extrinsic components of displacement. The extrinsic displacement components
         * are then added to the previous position to set the new coordinates
         */
        robotX += deltaX * Math.sin(Math.toRadians(robotAngle)) + deltaY * Math.cos(Math.toRadians(robotAngle));
        robotY += deltaX * -Math.cos(Math.toRadians(robotAngle)) + deltaY * Math.sin(Math.toRadians(robotAngle));


        // Set last encoder values for next loop
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();
    }


    double subtractAngles(double first, double second)
    {
        double delta = first - second;
        while(delta > 180)
            delta -= 360;
        while(delta <= -180)
            delta += 360;
        return delta;
    }
//------------------------------------------------------------------------------------------------------------------------------------
//Color Sensor/Aligning methods
    void colorSensorHSV()
    {
        // convert the RGB values to HSV values.
        Color.RGBToHSV((sensorTopLeft.red() * 255) / 800, (sensorTopLeft.green() * 255) / 800, (sensorTopLeft.blue() * 255) / 800, hsvValuesTopLeft);
        Color.RGBToHSV((sensorTopRight.red() * 255) / 800, (sensorTopRight.green() * 255) / 800, (sensorTopRight.blue() * 255) / 800, hsvValuesTopRight);
        Color.RGBToHSV((sensorBottomRight.red() * 255) / 800, (sensorBottomRight.green() * 255) / 800, (sensorBottomRight.blue() * 255) / 800, hsvValuesBottomRight);


        //Send current info back to driver
        //telemetry.addData("H: ", hsvValuesTopRight[0]);
        telemetry.addData("TopRight_S: ", hsvValuesTopRight[1]);
        //telemetry.addData("V: ", hsvValuesTopLeft[2]);
        telemetry.addData("TopLeft_S: ", hsvValuesTopLeft[1]);
        telemetry.addData("BottomRight_S: ", hsvValuesBottomRight[1]);
        telemetry.update();
    }

    void MoveIMUCont(double referenceAngle, double kAngle, double maxSpeed, double saturationValue)
    {
        runtime.reset();
        do
        {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            angleError = currentRobotAngle - referenceAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            motorPowerFL = maxSpeed + pivot;
            motorPowerFR = -maxSpeed + pivot;
            motorPowerBL = maxSpeed + pivot;
            motorPowerBR = -maxSpeed + pivot;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);

            Color.RGBToHSV((sensorTopLeft.red() * 255) / 800, (sensorTopLeft.green() * 255) / 800, (sensorTopLeft.blue() * 255) / 800, hsvValuesTopLeft);
            Color.RGBToHSV((sensorTopRight.red() * 255) / 800, (sensorTopRight.green() * 255) / 800, (sensorTopRight.blue() * 255) / 800, hsvValuesTopRight);
            Color.RGBToHSV((sensorBottomRight.red() * 255) / 800, (sensorBottomRight.green() * 255) / 800, (sensorBottomRight.blue() * 255) / 800, hsvValuesBottomRight);

            idle();
        }
        while ((opModeIsActive()) && (hsvValuesTopRight[1] <= saturationValue) && (hsvValuesTopLeft[1] <= saturationValue) && (hsvValuesBottomRight[1] <= saturationValue));
        stopDriving();
    }

    void driveOmni45Cont(double driveAngle, double drivePower, double turnPower, double saturationValue, double timeout)
    {
        runtime.reset();
        do
            {
                // Calculate out x and y directions of drive power, where y is forward (0 degrees) and x is right (-90 degrees)
                double x = drivePower * -Math.sin(Math.toRadians(driveAngle));
                double y = drivePower * Math.cos(Math.toRadians(driveAngle));

        /*
         * to explain:
         * each wheel omni wheel exerts a force in one direction like tank but differs in the fact that they have rollers mounted perpendicular
         * to the wheels outer edge so they can passively roll at 90 degrees to the wheel's facing. This means that with the wheels mounted at 45
         * degrees to the chassis frame and assuming the left hand rule for the motors, each wheel's power needs to be 90 degrees out of phase
         * from the previous wheel on the unit circle starting with positive y and x for FL and going clockwise around the unit circle and robot
         * from there
         */

                double powerFL = y + x + (turnPower * 0.9);
                double powerFR = -y + x + (turnPower * 0.9);
                double powerBL = y - x + (turnPower * 0.9);
                double powerBR = -y - x + (turnPower * 0.9);

                double scalar = Math.max(Math.abs(powerFL),  Math.max(Math.abs(powerFR),
                        Math.max(Math.abs(powerBL), Math.abs(powerBR))));

                if(scalar < 1)
                    scalar = 1;

                powerFL /= (scalar * slowModeDivisor);
                powerFR /= (scalar * slowModeDivisor);
                powerBL /= (scalar * slowModeDivisor);
                powerBR /= (scalar * slowModeDivisor);

                motorFL.setPower(powerFL);
                motorFR.setPower(powerFR);
                motorBL.setPower(powerBL);
                motorBR.setPower(powerBR);

            }
        while ((opModeIsActive()) && (runtime.seconds() < timeout));
        stopDriving();
    }

    // Moves continuosly left while no sensors are on the line
    void MoveIMUContLeft(double referenceAngle, double kAngle, double maxSpeed, double saturationValue, double timeout)
    {
        runtime.reset();
        do
        {
            currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle
            angleError = currentRobotAngle - referenceAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            motorPowerFL = -maxSpeed + pivot;
            motorPowerFR = -maxSpeed + pivot;
            motorPowerBL = maxSpeed + pivot;
            motorPowerBR = maxSpeed + pivot;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);

            Color.RGBToHSV((sensorTopLeft.red() * 255) / 800, (sensorTopLeft.green() * 255) / 800, (sensorTopLeft.blue() * 255) / 800, hsvValuesTopLeft);
            Color.RGBToHSV((sensorTopRight.red() * 255) / 800, (sensorTopRight.green() * 255) / 800, (sensorTopRight.blue() * 255) / 800, hsvValuesTopRight);
            Color.RGBToHSV((sensorBottomRight.red() * 255) / 800, (sensorBottomRight.green() * 255) / 800, (sensorBottomRight.blue() * 255) / 800, hsvValuesBottomRight);
            idle();
        }
        while ((opModeIsActive()) && (hsvValuesTopRight[1] <= saturationValue) && (hsvValuesTopLeft[1] <= saturationValue) && (hsvValuesBottomRight[1] <= saturationValue) && (runtime.seconds() < timeout));
        stopDriving();
    }

    // Method aligns on line with series of translations based on what sensors are on the line
    void alignOnLine55(double saturationValue, double timeout, double speed)
    {
        //Go forwards until any sensor sees the line
        double referenceAngle = imu.getAngularOrientation().firstAngle;//TODO declare this in init hardware auto
        double runTimes = 0.0;
        runtime.reset();

            telemetry.addData("Stage", "Zero");
            telemetry.update();
            runTimes ++;
            colorSensorHSV();

            //If no sensors are on the line, go forward
            while ((opModeIsActive()) && (hsvValuesTopRight[1] < saturationValue) && (hsvValuesTopLeft[1] < saturationValue) && (hsvValuesBottomRight[1] < saturationValue))
            {
                //MoveIMUCont(referenceAngle, 0.015, speed, saturationValue);
                MoveIMU(referenceAngle, 190.0, 0.0, 0.015, speed, 0.2);
                colorSensorHSV();
            }
            stopDriving();
            colorSensorHSV();

            //Top right sensor is on the line, but rest aren't
            if ((hsvValuesTopRight[1] > saturationValue) && (hsvValuesBottomRight[1] < saturationValue) && (hsvValuesTopLeft[1] < saturationValue))
            {
                telemetry.addData("Stage", "One");
                telemetry.update();
                //Drive at 55 degrees until the left sensor is on the line
                while ((opModeIsActive()) && hsvValuesTopLeft[1] < saturationValue)
                {
                    driveOmni45Cont(-55, speed, 0, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                stopDriving();
            }

            //Right sensors are on the line, but left isn't
            else if ((hsvValuesTopRight[1] > saturationValue) && (hsvValuesBottomRight[1] > saturationValue) && (hsvValuesTopLeft[1] < saturationValue))
            {
                telemetry.addData("Stage", "Two");
                telemetry.update();
                //Drive at 55 degrees until the left sensor is on the line
                while ((opModeIsActive()) && (hsvValuesTopLeft[1] < saturationValue))
                {
                    driveOmni45Cont(-55, speed, 0, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                stopDriving();
                colorSensorHSV();
            }

            //Only top left sensor sees line
            else if ((hsvValuesTopRight[1] < saturationValue) && (hsvValuesBottomRight[1] < saturationValue) && (hsvValuesTopLeft[1] > saturationValue))
            {
                telemetry.addData("Stage", "Three");
                telemetry.update();
                //Drive left until the right sensors see the line
                while ((opModeIsActive()) && (hsvValuesTopRight[1] < saturationValue) && (hsvValuesBottomRight[1] < saturationValue))
                {
                    MoveIMUContLeft(referenceAngle, 0, speed, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                stopDriving();

                //Drive at 55 degrees until the left sensor is on the line
                while ((opModeIsActive()) && (hsvValuesTopLeft[1] < saturationValue))
                {
                    driveOmni45Cont(-55, speed, 0, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                stopDriving();
            }

            //Top left and right sensor on line
            else if ((hsvValuesTopRight[1] > saturationValue) && (hsvValuesBottomRight[1] < saturationValue) && (hsvValuesTopLeft[1] > saturationValue))
            {
                telemetry.addData("Stage", "Four");
                telemetry.update();
                while ((opModeIsActive()) && (hsvValuesBottomRight[1] < saturationValue))
                {
                    MoveIMUContLeft(referenceAngle, 0, speed, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                stopDriving();
                //Drive at 55 degrees until the left sensor is on the line
                while ((opModeIsActive()) && (hsvValuesTopLeft[1] < saturationValue))
                {
                    driveOmni45Cont(-55, speed, 0, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                stopDriving();
            }
            else
            {
                while ((opModeIsActive()) && (hsvValuesTopLeft[1] < saturationValue)) {
                    driveOmni45Cont(-55, speed, 0, saturationValue, 0.1);
                    sleep(300);
                    colorSensorHSV();
                }
                telemetry.addData("Stage", "4.5");
                telemetry.update();
                stopDriving();
                sleep(300);
            }
        stopDriving();
        telemetry.addData("Stage", "Five");
        telemetry.update();
    }

//-------------------------------------------------------------------------------------------------------------------------------------
//Vuforia Methods

    //Method returns which vuMark the camera sees
    public RelicRecoveryVuMark GetVumark()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    //Decides if pixel is red, blue or other
    public float countPixel(float hue)
    {
        if (hue >= 333 || hue <= 20)//Range of Red Hue
        {
            numRedPixels ++;//Adds to num of red pixels
        }
        else if (hue >= 200 && hue <= 270)//Range of blue hue value
        {
            numBluePixels ++;//Adds to num of blue pixels
        }
        else//If the pixel value is not red or blue, it is identified as other.
        {
            numOtherPixels ++;//Adds to num of other pixels
        }
        return hue;
    }

    //Find avg Hue
    public float GetAvgJewelColor(int x, int y)
    {
        if (x>=0 && x<1280-32 && y>=0 && y<720-32)
        {
            HsvSum[0] = 0.0f;
            for (int j = y - 32; j < y + 32; j++) // "Draws" the columns
            {
                for (int i = x - 32; i < x + 32; i++) // "Draws" the rows
                {
                    // gets RGB color of the pixel
                    color = bm.getPixel(i, j);

                    /* convert RGB to HSV - hue, sat, val
                    The hue determines color in a 360 degree circle: 0 red, 60 is yellow, 120 is green
                    , 180 is cyan, 240 is blue, 300 is magenta
                    */
                    Color.colorToHSV(color, HSV);

                    // Adds the HSV color of all pixels
                    HsvSum[0] += HSV[0];

                    // draws a 64 by 64 black border around sample region
                    //(For debugging code only)
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31)) {
                        bm.setPixel(i, j, 0xff0000ff);//Blue Color()
                    }
                }
            }
            // Averages the HSV by dividing by 4096(64*64)
            HsvOut[0] = HsvSum[0] / 4096;
        }
        return HsvOut[0]; // returns the now averaged sampled HSV color value

    }

    //Count pixels with red, blue, and other hues & returns if left jewel is red
    public boolean GetJewelColor(int x, int y)
    {
        //Zero values since function gets called mult times
        numRedPixels = 0;
        numBluePixels = 0;
        numOtherPixels = 0;

        if (x >= 0 && x < 1280 - 32 && y >= 0 && y < 720 - 32) {
            for (int j = y - 32; j < y + 32; j++) // "Draws" the columns
            {
                for (int i = x - 32; i < x + 32; i++) // "Draws" the rows
                {
                    // gets RGB color of the pixel
                    color = bm.getPixel(i, j);

                    /* convert RGB to HSV - hue, sat, val
                    The hue determines color in a 360 degree circle: 0 red, 60 is yellow, 120 is green
                    , 180 is cyan, 240 is blue, 300 is magenta
                    */
                    Color.colorToHSV(color, HSV);

                    // Determine and count red, blue, other hue pixels. Counts are in numRedPixels, numBluePixels, numOtherPixels vars
                    countPixel(HSV[0]);

                    // draws a 64 by 64 black border around sample region
                    //(For debugging code only)
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31)) {
                        bm.setPixel(i, j, 0xff0000ff);//Blue Color()
                    }
                }
            }
            //Compare color counts
            if (numRedPixels > numBluePixels) isLeftJewelRed = true;
            else isLeftJewelRed = false;
        }
        return isLeftJewelRed; // returns if left jewel is red
    }

    public boolean GetLeftJewelColorCount() throws InterruptedException
    {
        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();
        Vec2F leftJewel;
        if (pose!=null) {
            Matrix34F rawPose = new Matrix34F();
            //rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            //Projects point at
            //Right: (390, -180, -102)
            //Left: (165, -175, -102)
            Vec2F rightJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(390, -180, -102));
            leftJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(165, -175, -102));
        }
        else
        {
            // set default pixel location of left jewel is VuMark isn't visible
            leftJewel = new Vec2F(781.0f,509.0f);
        }
            // Takes a frame
            frame = vuforia.getFrameQueue().take();

            long numberImages = frame.getNumImages();

            for (int j = 0; j < numberImages; j++)
            {
                image = frame.getImage(j);
                imageFormat = image.getFormat();
                if (imageFormat == PIXEL_FORMAT.RGB565) break;
            }

            int imageWidth = image.getWidth(); // Gets width of image
            int imageHeight = image.getHeight(); // Gets height of image

            //Creates bitmap of the image to detect color of jewels
            bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(image.getPixels());

            //Declare int for left jewels
            int LeftX = (int) leftJewel.getData()[0];
            int LeftY = (int) leftJewel.getData()[1];

            // Use method for getting Jewel color (seen above)
            GetJewelColor(LeftX, LeftY);

        return isLeftJewelRed;
    }

    public void initVuforia()
    {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        //This licence key belongs to Steve Geffner
        parameters.vuforiaLicenseKey = "ATJf0AL/////AAAAGQZ9xp9L+k5UkmHj3LjxcoQwNTTBJqjO9LYsbkWQ" +
                "ArRpYKQmt7vqe680RCQSS9HatStn1XZVi7rgA8T7qrJz/KYI748M4ZjlKv4Z11gryemJCRA9+WWkQ51" +
                "D3TuYJbQC46+LDeMfbvcJQoQ79jtXr7xdFhfJl1mRxf+wMVoPWfN6Dhr8q3XVxFwOE/pM3gXWQ0kacb" +
                "cGR/vy3NAsbOhf02DEe5WoV5PNZTF34LWN3dWURu7NJsnbFzkpzXdogeVAdiQ3QUWDvuhEwvSJY4W+f" +
                "CTb15t6T/c/GJ/vqptsVKqavXk6MQobnUsVFpFP+5OSuRQe7EgvWuOxn7xn5YlC+CWAYh9LrXDpktwC" +
                "wBAiX3Gx";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        // set phone location
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(0.0f, 0.0f, 0.0f)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.XYZ,
                        AngleUnit.DEGREES, 90, -90, 0));  // landscape back camera

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(1); // this tells VuforiaLocalizer to only store one frame at a time
        // wait until the start button is pressed

        relicTrackables.activate(); // activate tracking
    }
}
