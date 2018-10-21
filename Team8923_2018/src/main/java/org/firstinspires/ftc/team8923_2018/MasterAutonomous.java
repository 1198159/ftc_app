package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

abstract class MasterAutonomous extends Master
{
    // robot's position and angle on the field tracked in these variables
    double robotX;
    double robotY;
    double robotAngle;
    double headingOffset = 0.0;

    // Used to calculate distance traveled between loops
    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    Alliance alliance = Alliance.BLUE;
    StartLocations startLocation = StartLocations.DEPOT;
    boolean doneSettingUp = false;

    // these values equal to one over the value (in mm for drive power and degrees for turn power)
    // that you want the PID loop to start limiting the speed at
    double DRIVE_POWER_CONSTANT = 1/250; // start slowing down at 750 millimeters from the target location
    double TURN_POWER_CONSTANT = 1/35; // start slowing down at 35 degrees away from the target angle;

    double MIN_DRIVE_POWER = 0.3; // don't let the robot go slower than this speed
    int TOL = 100;
    OpenCV openCVVision = new OpenCV();
    enum Alliance
    {
        BLUE,
        RED
    }
    enum StartLocations
    {
        DEPOT(0),
        CRATER(0),

        RED_DEPOT_START_X(0),
        RED_DEPOT_START_Y(0),
        RED_DEPOT_START_ANGLE(0),

        BLUE_DEPOT_START_X(0),
        BLUE_DEPOT_START_Y(0),
        BLUE_DEPOT_START_ANGLE(0),

        RED_CRATER_START_X(0),
        RED_CRATER_START_Y(0),
        RED_CRATER_START_ANGLE(0),

        BLUE_CRATER_START_X(0),
        BLUE_CRATER_START_Y(0),
        BLUE_CRATER_START_ANGLE(0);

        public final double val;
        StartLocations(double i) {val = i;}
    }

    public void initAuto()
    {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initHardware();
        telemetry.addData("Init State", "Init Finished");
        telemetry.update();

        // Set last known encoder values
        lastEncoderFL = motorFL.getCurrentPosition();
        lastEncoderFR = motorFR.getCurrentPosition();
        lastEncoderBL = motorBL.getCurrentPosition();
        lastEncoderBR = motorBR.getCurrentPosition();


        // Set IMU heading offset
        headingOffset = imu.getAngularOrientation().firstAngle - robotAngle;

        telemetry.clear();
        telemetry.update();
        telemetry.log().add("Initialized. Ready to start!");
    }

    public void moveLift (int ticks)
    {
        while ((ticks - motorLift.getCurrentPosition()) > TOL)
        {
            motorLift.setTargetPosition(motorLift.getCurrentPosition() + ticks);
            motorLift.setPower((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 1000.0));
            idle();;
        }
        stopLift();
    }
    public void stopLift()
    {
        motorLift.setPower(0.0);
        idle();
    }
    public void moveJJ(int position)
    {
        //1 is completely up
        //-1 is completely down
        servoJJ.setPosition(position);
    }

    public void configureAutonomous()
    {
        telemetry.log().add("Alliance Blue/Red: X/B");
        telemetry.log().add("Starting Position Crater/Depot: D-Pad Up/Down");
        telemetry.log().add("");
        telemetry.log().add("After routine is complete and robot is on field, press Start");

        while(!doneSettingUp)
        {
            if(gamepad1.x)
                alliance = Alliance.BLUE;
                //means we are blue alliance
            else if (gamepad1.b)
                alliance = Alliance.RED;
                // means we are red alliance

            if(gamepad1.dpad_up)
            {
                startLocation = StartLocations.CRATER;
            }
                //means we are crater side
            else if (gamepad1.dpad_down)
            {
                startLocation = StartLocations.DEPOT;
            }
                //means we are depot side

            if(gamepad1.start)
                doneSettingUp = true;

            while (!buttonsAreReleased(gamepad1))
            {
                idle();
                telemetry.update();
            }

            telemetry.addData("Alliance", alliance.name());
            telemetry.addData("Side", startLocation.name());
            telemetry.update();
            idle();
        }

        // We could clear the telemetry at this point, but the drivers may want to see it
        telemetry.clear();
        telemetry.update();

        telemetry.log().add("Setup complete. Initializing...");

        // Set coordinates based on alliance and starting location
        if(startLocation == StartLocations.DEPOT)
        {
            if(alliance == Alliance.RED)
            {
                robotX = StartLocations.RED_DEPOT_START_X.val;
                robotY = StartLocations.RED_DEPOT_START_Y.val;
                robotAngle = StartLocations.RED_DEPOT_START_ANGLE.val;
            }
            else if(alliance == Alliance.BLUE)
            {
                robotX = StartLocations.BLUE_DEPOT_START_X.val;
                robotY = StartLocations.BLUE_DEPOT_START_Y.val;
                robotAngle = StartLocations.BLUE_DEPOT_START_ANGLE.val;
            }
        }
        else if(startLocation == StartLocations.CRATER)
        {
            if (alliance == Alliance.RED)
            {
                robotX = StartLocations.RED_CRATER_START_X.val;
                robotY = StartLocations.RED_CRATER_START_Y.val;
                robotAngle = StartLocations.RED_CRATER_START_ANGLE.val;
            } else if (alliance == Alliance.BLUE)
            {
                robotX = StartLocations.BLUE_CRATER_START_X.val;
                robotY = StartLocations.BLUE_CRATER_START_Y.val;
                robotAngle = StartLocations.BLUE_CRATER_START_ANGLE.val;
            }
        }
    }

    // Makes robot drive to a point on the field
    void driveToPoint(double targetX, double targetY, double targetAngle, double maxPower) throws InterruptedException
    {
        updateRobotLocation();

        // Calculate how far we are from target point
        double distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);
        double deltaAngle = subtractAngles(targetAngle, robotAngle);
        double DISTANCE_TOLERANCE = 40; // In mm
        double ANGLE_TOLERANCE = 5; // In degrees

        // Run until robot is within tolerable distance and angle
        while(!(distanceToTarget < DISTANCE_TOLERANCE && deltaAngle < ANGLE_TOLERANCE) && opModeIsActive())
        {
            updateRobotLocation();

            // In case robot drifts to the side
            double driveAngle = Math.toDegrees(Math.atan2(targetY - robotY, targetX - robotX)) - robotAngle;

            // Decrease power as robot approaches target. Ensure it doesn't exceed power limits
            double drivePower = Range.clip(distanceToTarget * DRIVE_POWER_CONSTANT, MIN_DRIVE_POWER, maxPower);

            // In case the robot turns while driving
            deltaAngle = subtractAngles(targetAngle, robotAngle);
            double turnPower = deltaAngle * TURN_POWER_CONSTANT;

            // Set drive motor powers
            driveMecanum(driveAngle, drivePower, turnPower);

            // Recalculate distance for next check
            distanceToTarget = calculateDistance(targetX - robotX, targetY - robotY);

            // Inform drivers of robot location
            telemetry.addData("X", robotX);
            telemetry.addData("Y", robotY);
            telemetry.addData("Robot Angle", robotAngle);
            telemetry.update();
            idle();
        }
        stopDriving();
    }

    // Updates robot's coordinates and angle
    void updateRobotLocation()
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
        double deltaX = (deltaFL - deltaFR - deltaBL + deltaBR) / 4;
        double deltaY = (deltaFL + deltaFR + deltaBL + deltaBR) / 4;

        // Convert to mm
        deltaX *= MM_PER_TICK;
        deltaY *= MM_PER_TICK;

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

    public void stopDriving ()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    public String getGoldPosition()
    {
        openCVVision.setShowCountours(true);
        String position;

        if(((OpenCV.getGoldRect().y + OpenCV.getGoldRect().height / 2) < 150) && (OpenCV.getGoldRect().y + OpenCV.getGoldRect().height / 2) > 0)
        {
            telemetry.addData("Position: ", "Left");
            position = "Left";
        }
        else if((OpenCV.getGoldRect().y + OpenCV.getGoldRect().height / 2) > 250)
        {
            telemetry.addData("Position: ", "Center");
            position = "Center";
        }
        else
        {
            telemetry.addData("Position", "Right");
            position = "Right";
        }
        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (OpenCV.getGoldRect().x + OpenCV.getGoldRect().width) / 2, (OpenCV.getGoldRect().y + OpenCV.getGoldRect().height) / 2));
        telemetry.update();

        return position;
    }
}
