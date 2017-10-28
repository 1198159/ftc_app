package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Holds all code necessary to run the robot in driver controlled mode
 */

public abstract class MasterTeleOp extends Master
{
    int GGLiftTicks = 250;

    boolean slowModeActive = false;
    boolean liftMoving = false;

    double robotAngle;
    double anglePivot;

    double jy;
    double jx;
    double error;
    double pivot;
    double kAngle;
    double jpivot;//for pivoting

    ElapsedTime GGLiftTimer = new ElapsedTime();


    void DriveOmni45TeleOp()
    {
        if(gamepad1.guide)
        {
            if(slowModeActive)
                slowModeDivisor = 1.0;
            else if(!slowModeActive)
                slowModeDivisor = 5.0;

            slowModeActive = !slowModeActive;
        }

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;
        double power = calculateDistance(x, y);
        double turnPower = -gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveOmni45(angle, power, turnPower);
    }

    /*void IMUDrive()//Drive for straighter movement. However, pivot is very slow For now, maybe don't use this method in TeleOp until faster pivot is made
    {
        jy = -gamepad1.left_stick_y;
        jx = gamepad1.left_stick_x;
        jpivot = gamepad1.right_stick_x;
        anglePivot = 2 * (anglePivot - jpivot);
        anglePivot = adjustAngles(anglePivot);


        kAngle = 0.035;
        robotAngle = imu.getAngularOrientation().firstAngle;
        //robotAngle = adjustAngles(robotAngle);
        error = robotAngle - anglePivot;
        error = adjustAngles(error);
        pivot = error * kAngle;
        motorPowerFL = jx - jy - pivot;
        motorPowerFR = -jx + jy - pivot;
        motorPowerBL = jx + jy - pivot;
        motorPowerBR = -jx - jy - pivot;

        motorFL.setPower(motorPowerFL);
        motorFL.setPower(motorPowerFR);
        motorBL.setPower(motorPowerBL);
        motorBR.setPower(motorPowerBR);
    }
    */
    void RunGG()
    {
        if((gamepad1.dpad_down || gamepad1.dpad_up) && !liftMoving)
        {
            liftMoving = true;
            GGLiftTimer.reset();

            if (gamepad1.dpad_up)
                motorGG.setTargetPosition(motorGG.getCurrentPosition() + GGLiftTicks);

            else if (gamepad1.dpad_down)
                motorGG.setTargetPosition(motorGG.getCurrentPosition() - GGLiftTicks);

            motorGG.setPower(Math.signum(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * 0.75);
        }

        if(GGLiftTimer.milliseconds() > 250 && Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) < 50)
        {
            motorGG.setPower(0.0);
            liftMoving = false;
        }

        if(gamepad1.x && !liftMoving)
        {
            servoGGL.setPosition(0.0); //TODO value needs to be changed
            servoGGR.setPosition(0.0); //TODO value to be changed

        }
         else if(gamepad1.b && !liftMoving)
        {
            servoGGL.setPosition(0.0);//TODO value needs to be changed
            servoGGR.setPosition(0.0);//TODO value needs to be changed
        }
    }
}
