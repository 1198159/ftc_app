package org.firstinspires.ftc.team8923_2017;

/*
 * Holds all code necessary to run the robot in autonomous controlled mode
 */

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
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
    double SERVO_JJ_MIDDLE1 = 0.45;
    double SERVO_JJ_MIDDLE2 = 0.4;
    double SERVO_JJ_MIDDLE3 = 0.35;
    double SERVO_JJ_MIDDLE4 = 0.3;
    double SERVO_JJ_MIDDLE5 = 0.25;
    double SERVO_JJ_MIDDLE6 = 0.2;

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
    //Matrix34F rawPose;
    //Matrix34F rawPose = new Matrix34F();


    public float leftColorHSV[] = {0f, 0f, 0f};
    public float rightColorHSV[] = {0f, 0f, 0f};

    int color = 0;
    float[] HsvSum = {0,0,0};
    float[] HSV = {0,0,0};
    float[] HsvOut = {0,0,0};
    float avgLeftJewelColor;
    float avgRightJewelColor;

    float Leftcolor;
    float Rightcolor;
    float deltaHSVColor;

    boolean isVuMarkVisible;
    boolean isLeftJewelRed;

    VuforiaLocalizer vuforia;


    OpenGLMatrix pose;


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

    void InitAuto()
    {
        InitHardware();

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

        robotAngle = 90;

        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;
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
        servoGGL.setPosition(0.5);
        servoGGR.setPosition(0.1);
    }

    void MoveIMU(double referenceAngle, double moveMM, double targetAngle, double kAngle, double maxSpeed, double timeout)
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
            motorPowerFL = -speedBR + pivot;
            motorPowerFR = speedBR + pivot;
            motorPowerBL = -speedBR + pivot;
            motorPowerBR = speedBR + pivot;

            //Sets motor power
            //motorFL.setPower(0.5);
            //motorFR.setPower(-0.5);
            //motorBL.setPower(0.5);
            //motorBR.setPower(-0.5);
            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
            //sleep(100);

            //motorFL.setPower(0.0);
            //motorFR.setPower(0.0);
            //motorBL.setPower(0.0);
            //motorBR.setPower(0.0);

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

    void move(double moveMM, double maxSpeed, double timeout)
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

            //Sets values for motor power
            motorPowerFL = speedFL;
            motorPowerFR = speedFR;
            motorPowerBL = speedBL;
            motorPowerBR = speedBR;

            motorFL.setPower(-motorPowerBR);
            motorFR.setPower(motorPowerBR);
            motorBL.setPower(-motorPowerBR);
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

    void pivot(double moveMM, double maxSpeed, double timeout)
    {
        currentFL = motorFL.getCurrentPosition();
        currentFR = motorFR.getCurrentPosition();
        currentBL = motorBL.getCurrentPosition();
        currentBR = motorBR.getCurrentPosition();

        //Sets motor encoder values
        newTargetFL = motorFL.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetFR = motorFR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetBL = motorBL.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);
        newTargetBR = motorBR.getCurrentPosition() - (int) (moveMM / MM_PER_TICK);

        runtime.reset(); // used for timeout

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

            //Sets values for motor power
            motorPowerFL = speedFL;
            motorPowerFR = speedFR;
            motorPowerBL = speedBL;
            motorPowerBR = speedBR;

            motorFL.setPower(motorPowerFL);
            motorFR.setPower(motorPowerFR);
            motorBL.setPower(motorPowerBL);
            motorBR.setPower(motorPowerBR);
            idle();
        }
        while (opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (Math.abs(moveErrorFL) > TOL));

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
            //Slows down to allow IMU to catch up
            /*if (Math.abs(angleError) < 5.0)
            {
                sleep(30);
                motorFL.setPower(0);
                motorFR.setPower(0);
                motorBL.setPower(0);
                motorBR.setPower(0);
                sleep(150);
            }
            */
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


    void DropJJ()
    {
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
        servoJJ.setPosition(SERVO_JJ_MIDDLE6);
        sleep(200);
        servoJJ.setPosition(SERVO_JJ_DOWN);
    }

    void RetrieveJJ()
    {
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

//-------------------------------------------------------------------------------------------------------------------------------------
//Vuforia Methods

    public RelicRecoveryVuMark GetVumark()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }


    public float GetAvgJewelColor(int x, int y)
    {
        if (x>=0 && x<1280-32 && y>=0 && y<720-32)
        {
            HsvSum[0] = 0;
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
                    if ((j == y - 32) || (j == y + 31) || (i == x - 32) || (i == x + 31))
                    {
                        bm.setPixel(i, j, 0xff0000ff);//Blue Color()
                    }
                }
            }
            // Averages the HSV by dividing by 4096(64*64)
            HsvOut[0] = HsvSum[0] / 4096;
        }
        return HsvOut[0]; // returns the now averaged sampled HSV color value
    }


    public boolean GetLeftJewelColor() throws InterruptedException
    {
        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

        if (pose!=null)
        {
            Matrix34F rawPose = new Matrix34F();
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            //Projects point at
            //Right: (390, -180, -102)
            //Left: (165, -175, -102)
            Vec2F rightJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(390, -180, -102));
            Vec2F leftJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(165, -175, -102));

            // Takes a frame
            frame = vuforia.getFrameQueue().take();

            long numberImages = frame.getNumImages();

            for (int j = 0; j < numberImages; j++)
            {
                image = frame.getImage(j);
                imageFormat = image.getFormat();
                if (imageFormat == PIXEL_FORMAT.RGB565) break;
            }

            int imageWidth = image.getWidth();
            int imageHeight = image.getHeight();

            //Creates bitmap of the image to detect color of jewels
            bm = Bitmap.createBitmap(imageWidth, imageHeight, Bitmap.Config.RGB_565);
            bm.copyPixelsFromBuffer(image.getPixels());

            //Declare int for left jewels
            int LeftX = (int) leftJewel.getData()[0];
            int LeftY = (int) leftJewel.getData()[1];

            //Declares int for right jewels
            int RightX = (int) rightJewel.getData()[0];
            int RightY = (int) rightJewel.getData()[1];

            avgLeftJewelColor = GetAvgJewelColor(LeftX, LeftY); // gets the averaged jewel HSV color value for the left jewel
            avgRightJewelColor = GetAvgJewelColor(RightX, RightY);//Gets the averaged jewel HSV color value for the right jewel

            //adjusts color for red so that red is greater that blue by adding 300 since red is only 45
            Leftcolor = (avgLeftJewelColor < 45) ? avgLeftJewelColor + 300 : avgLeftJewelColor;
            Rightcolor = (avgRightJewelColor < 45) ? avgRightJewelColor + 300 : avgRightJewelColor;
        }
        //Gets the difference between left HSV and right HSV
        deltaHSVColor = Leftcolor - Rightcolor;
        // if the left jewel color is positive, the left side is red and the right side is blue
        if (deltaHSVColor > 0) isLeftJewelRed = true;
        else isLeftJewelRed = false;
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
        parameters.vuforiaLicenseKey = "ATJf0AL/////AAAAGQZ9xp9L+k5UkmHj3LjxcoQwNTTBJqjO9LYsbkWQArRpYKQmt7vqe680RCQSS9HatStn1XZVi7rgA8T7qrJz/KYI748M4ZjlKv4Z11gryemJCRA9+WWkQ51D3TuYJbQC46+LDeMfbvcJQoQ79jtXr7xdFhfJl1mRxf+wMVoPWfN6Dhr8q3XVxFwOE/pM3gXWQ0kacbcGR/vy3NAsbOhf02DEe5WoV5PNZTF34LWN3dWURu7NJsnbFzkpzXdogeVAdiQ3QUWDvuhEwvSJY4W+fCTb15t6T/c/GJ/vqptsVKqavXk6MQobnUsVFpFP+5OSuRQe7EgvWuOxn7xn5YlC+CWAYh9LrXDpktwCwBAiX3Gx";

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
