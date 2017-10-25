package org.firstinspires.ftc.team8923_2017;

/*
 * Holds all code necessary to run the robot in autonomous controlled mode
 */

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
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

    Alliance alliance = Alliance.RED;
    StartPositions startPosition = StartPositions.LEFT;
    boolean setupFinished = false;
    int delayTime = 0;

    double robotX;
    double robotY;
    double robotAngle;

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
    Matrix34F rawPose;

    // old color variables for left and right jewels
    public float leftColorHSV[] = {0f, 0f, 0f};
    public float rightColorHSV[] = {0f, 0f, 0f};

    int color = 0;
    float[] colorHsvSum = {0,0,0};
    float[] colorHSV = {0,0,0};
    float[] colorHsvOut = {0,0,0};
    float avgLeftJewelColor;
    float avgRightJewelColor;

    float colorLeft;
    float colorRight;
    float deltaColorHSV;

    boolean isVuMarkVisible;
    boolean isLeftJewelBlue;

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

    void MoveIMU(double moveMM, double targetAngle, double kAngle, double maxSpeed, double timeout)
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

            targetAngle = adjustAngles(targetAngle);//Makes it so target angle does not wrap
            angleError = currentRobotAngle - targetAngle;
            angleError = adjustAngles(angleError);
            pivot = angleError * kAngle;

            //Sets values for motor power
            motorPowerFL = speedFL + pivot;
            motorPowerFR = speedFR + pivot;
            motorPowerBL = speedBL + pivot;
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
                (Math.abs(moveErrorFL) > TOL));

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

    void pivotGoTo (double GoTO)
    {

    }

    void IMUPivot(double targetAngle, double maxSpeed, double kAngle)
    {
        currentRobotAngle = imu.getAngularOrientation().firstAngle;//Sets currentRobotAngle as the current robot angle

        targetAngle = adjustAngles(targetAngle);//Makes it so the target angle does not wrap
        angleError = currentRobotAngle - targetAngle;
        angleError = adjustAngles(angleError);
        pivot = angleError * kAngle;

        motorPowerFL = speedFL + pivot;
        motorPowerFR = speedFR + pivot;
        motorPowerBL = speedBL + pivot;
        motorPowerBR = speedBR + pivot;

        motorFL.setPower(motorPowerFL);
        motorFR.setPower(motorPowerFR);
        motorBL.setPower(motorPowerBL);
        motorBR.setPower(motorPowerBR);

        while (opModeIsActive() && (Math.abs(angleError) > TOL));

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
    }

    public RelicRecoveryVuMark GetVumark()
    {
        vuMark = RelicRecoveryVuMark.from(relicTemplate);
        return vuMark;
    }

    public boolean GetLeftJewelColor() throws InterruptedException
    {
        pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getRawPose();

        if (pose!=null)
        {
            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
            rawPose.setData(poseData);
            // image size is 254 mm x 184 mm
            //Project point at
            Vec2F leftJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(165, -175, -102));
            Vec2F rightJewel = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(390, -180, -102));
            // takes the frame at the head of the queue

            frame = vuforia.getFrameQueue().take();

            long numImages = frame.getNumImages();


        }

        return isLeftJewelBlue;
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
}
