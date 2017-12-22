package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Holds all code necessary to run the robot in driver controlled mode
 */

public abstract class MasterTeleOp extends Master
{
    boolean liftMoving = false;
    boolean liftModeStateChange = false;

    int liftStage = 0;

    double robotAngle;
    double anglePivot;

    double jy;
    double jx;
    double error;
    double pivot;
    double kAngle;
    double jPivot;//for pivoting

    int smallMovementsUp = 0;
    int smallMovementsDown = 0;
    int GGTOL = 15;

    ElapsedTime GGLiftTimer = new ElapsedTime();
    ElapsedTime SlowModeTimer = new ElapsedTime();


    void DriveOmni45TeleOp()
    {
        if(gamepad1.right_stick_button && SlowModeTimer.milliseconds() > 250)
        {
            SlowModeTimer.reset();
            if (slowModeDivisor == 1.0)
                slowModeDivisor = 3.0;
            else
                slowModeDivisor = 1.0;
        }

        double y = -gamepad1.left_stick_y; // Y axis is negative when up
        double x = gamepad1.left_stick_x;
        double power = calculateDistance(x, y);
        double turnPower = gamepad1.right_stick_x; // Fix for clockwise being a negative rotation

        double angle = Math.toDegrees(Math.atan2(-x, y)); // 0 degrees is forward

        driveOmni45(angle, power, turnPower);
        idle();
    }

    public void SendTelemetry()
    {
        telemetry.addData("GG Lift Ticks", motorGG.getCurrentPosition());
        telemetry.addData("GG distance from target", Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()));
        telemetry.addData("GG at position", Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) < 3);
        telemetry.addData("SlowMode", slowModeDivisor);
        //telemetry.addData("lift stage", liftStage);
        telemetry.update();
        idle();
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

        /*if(!liftMoving && (gamepad1.dpad_down || gamepad1.dpad_up))
        {
            liftMoving = true;
            GGLiftTimer.reset();

            if (gamepad1.dpad_up && liftStage < 2)
                liftStage++;

            else if (gamepad1.dpad_down && liftStage > 0)
                liftStage--;

            if((motorGG.getCurrentPosition() - GGLiftTicks < GGZero) && gamepad1.dpad_down)
                motorGG.setTargetPosition(GGZero);
            else if((motorGG.getCurrentPosition() + GGLiftTicks > GGZero + (GGLiftTicks * 2)) && gamepad1.dpad_up)
                motorGG.setTargetPosition(GGZero + (GGLiftTicks * 2));
        }

        if(liftMoving)
        {
            motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 125.0), 1.0));
        }

        if (GGLiftTimer.milliseconds() > 125 && Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 5)
        {
            motorGG.setPower(0.0);
            liftMoving = false;
        }*/


        if(!liftModeStateChange && !liftMoving && (gamepad1.dpad_up || gamepad1.dpad_down) && GGLiftTimer.milliseconds() > 500)
        {
            liftMoving = true;
            GGLiftTimer.reset();

            if(gamepad1.dpad_up && liftStage < 2)
            {
                liftStage++;
                motorGG.setTargetPosition(GGZero + (liftStage * GGLiftTicks));//1700
            }

            else if(gamepad1.dpad_down && liftStage > 0)
            {
                liftStage--;
                motorGG.setTargetPosition(GGZero + (liftStage * 1700));//1700
            }
        }
/*
        if(gamepad1.dpad_up && motorGG.getCurrentPosition() < 1700)
        {
            motorGG.setTargetPosition(1700);
        }
        else if(gamepad1.dpad_up && motorGG.getCurrentPosition() >= 1700)
        {
            motorGG.setTargetPosition(3400);
        }
        else if(gamepad1.dpad_down && motorGG.getCurrentPosition() >= 3400)
        {
            motorGG.setTargetPosition(1700);
        }
        else if(gamepad1.dpad_down && motorGG.getCurrentPosition() <= 3400)
        {
            if(gamepad1.dpad_down && motorGG.getCurrentPosition() <= 1700)
            {
                motorGG.setTargetPosition(0);
            }

            else
            {
                motorGG.setTargetPosition(1700);
            }
        }
*/
        if(liftMoving)
        {
            motorGG.setPower(Math.min(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 500.0), 0.6), 1.0));
        }

        if (GGLiftTimer.milliseconds() > 125 && Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
        {
            motorGG.setPower(0.0);
            liftMoving = false;
        }

        //full close
        if(gamepad1.a && !liftMoving)
        {
            servoGGL.setPosition(0.4); //TODO value needs to be changed
            servoGGR.setPosition(0.15); //TODO value to be changed
        }
        //half open
        else if(gamepad1.x && !liftMoving)
        {
            servoGGL.setPosition(0.25);//TODO value needs to be changed
            servoGGR.setPosition(0.25);//TODO value needs to be changed
        }
        //full open
        else if(gamepad1.b && !liftMoving)
        {
            servoGGL.setPosition(0.15);//TODO value needs to be changed
            servoGGR.setPosition(0.28);//TODO value needs to be changed
        }

        //if(liftModeStateChange && !(gamepad1.right_trigger > 0.35 || gamepad1.left_trigger > 0.35))
        if(liftModeStateChange && !(gamepad1.right_trigger > 0.35 || gamepad1.left_trigger > 0.35))
        {
            liftModeStateChange = false;
            motorGG.setTargetPosition(motorGG.getCurrentPosition());
            motorGG.setPower(0.0);
        }

        if(gamepad1.right_trigger > 0.35 && !liftMoving)
        {
            if(!liftModeStateChange)
            {
                // 3000 here just to set a target that's far enough out of the way so as not to worry about meeting it
                motorGG.setTargetPosition(motorGG.getCurrentPosition() + 3000);
                liftModeStateChange = true;
            }
            motorGG.setPower(0.25);
        }
        else if(gamepad1.left_trigger > 0.35 && !liftMoving)
        {
            if(!liftModeStateChange)
            {
                // 3000 here just to set a target that's far enough out of the way so as not to worry about meeting it
                motorGG.setTargetPosition(motorGG.getCurrentPosition() - 3000);
                liftModeStateChange = true;
            }
            motorGG.setPower(-0.25);
        }

        if(gamepad1.y)
        {
            GGZero = motorGG.getCurrentPosition();
        }
        idle();
    }

    public void RunGGLift()
    {
        liftMoving = true;
        liftModeStateChange = false;
        if(!liftModeStateChange && (gamepad1.right_bumper || gamepad1.left_bumper))
        {
            if(gamepad1.right_bumper && motorGG.getCurrentPosition() < GGZero +1700 - TOL)
            {
                motorGG.setTargetPosition(GGZero + 1700);
            }
            else if(gamepad1.right_bumper && motorGG.getCurrentPosition() >= GGZero +1700 - TOL)
            {
                motorGG.setTargetPosition(GGZero + 3400);
            }
            else if(gamepad1.left_bumper && motorGG.getCurrentPosition() >= GGZero +3400 - TOL)
            {
                motorGG.setTargetPosition(GGZero + 1700);
            }
            else if(gamepad1.left_bumper && motorGG.getCurrentPosition() <= GGZero +3400 + TOL)
            {
                if(gamepad1.left_bumper && motorGG.getCurrentPosition() <= GGZero +1700 + TOL)
                {
                    motorGG.setTargetPosition(GGZero + 40);
                }

                else
                {
                    motorGG.setTargetPosition(GGZero + 1650);
                }
            }
            motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 95.0), 1.0));
        }

        if (motorGG.getCurrentPosition() > GGZero + 3700)
        {
            motorGG.setPower(0.0);
            motorGG.setTargetPosition(GGZero + 3670);
            motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 125.0), 1.0));
        }


        if (Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
        {
            motorGG.setPower(0.0);
        }

        if(gamepad1.right_trigger > 0.35 || gamepad1.left_trigger > 0.35)
        {
            while(gamepad1.right_trigger > 0.35)
            {
                //if(!liftModeStateChange)
                //{
                motorGG.setTargetPosition(motorGG.getCurrentPosition() + 3000);
                liftModeStateChange = true;
                //}
                motorGG.setPower(0.25);
            }
            motorGG.setPower(0.0);
            liftModeStateChange = false;

            while(gamepad1.left_trigger > 0.35)
            {
                //if(!liftModeStateChange)
                //{
                motorGG.setTargetPosition(motorGG.getCurrentPosition() - 3000);
                liftModeStateChange = true;
                //}
                motorGG.setPower(-0.25);
            }
            motorGG.setPower(0.0);
            liftModeStateChange = false;
        }
        idle();
    }

    public void RunGGClaws()
    {
        if(gamepad1.a)
        {
            servoGGL.setPosition(0.4); //TODO value needs to be changed
            servoGGR.setPosition(0.15); //TODO value to be changed
        }
        //half open
        //else if(gamepad1.x && !liftMoving)
        else if(gamepad1.x)
        {
            servoGGL.setPosition(0.25);//TODO value needs to be changed
            servoGGR.setPosition(0.25);//TODO value needs to be changed
        }
        //full open
        //else if(gamepad1.b && !liftMoving)
        else if(gamepad1.b)
        {
            servoGGL.setPosition(0.15);//TODO value needs to be changed
            servoGGR.setPosition(0.28);//TODO value needs to be changed
        }
        if(gamepad1.left_stick_button && gamepad1.right_stick_button && gamepad1.left_bumper)
            GGZero = motorGG.getCurrentPosition() + 55;
        idle();
    }
}
