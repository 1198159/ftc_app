package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Holds all code necessary to run the robot in driver controlled mode
 */

public abstract class MasterTeleOp extends Master
{
    //region UNUSED VARIABLES
    /*boolean liftMoving = false;
    boolean RRExtended = false;
    boolean RRHandOpen = false;
    boolean RRMoving = false;
    boolean HandMoving = false;
    boolean RRAtPosition = true;

    int liftStage = 0;

    int smallMovementsUp = 0;
    int smallMovementsDown = 0;
    int GGTOL = 15;

    ElapsedTime GGLiftTimer = new ElapsedTime();
    ElapsedTime HandTimer = new ElapsedTime(); */
    //endregion

    private boolean liftModeStateChange = false;
    private boolean GGFlipped = false;
    private boolean GGLifted = false;
    private boolean bottomGGClosed = false;
    private boolean topGGClosed = false;
    private boolean firstTime = true; // used to reset GGFlipTimer on the first time through the statement
    private boolean autoBalanceStable = false;
    boolean autoBalancing = false;

    private int GGStart;
    private int GGFlipStage = 0;
    private int holdPosition;

    private double autoBalanceTargetAngle = 0;

    private ElapsedTime SlowModeTimer = new ElapsedTime();
    private ElapsedTime GGFlipTimer = new ElapsedTime();
    private ElapsedTime AutoBalanceTimeout = new ElapsedTime();

    enum GGServoPositions
    {
        LEFTFULLOPEN(0.65),
        RIGHTFULLOPEN(0.15),
        LEFTHALFOPEN(0.55),
        RIGHTHALFOPEN(0.35),
        LEFTFULLCLOSED(0.45),
        RIGHTFULLCLOSED(0.45);

        public final double val;
        GGServoPositions(double i) { val = i; }
        public final double val() { return val; }
    }

    void DriveOmni45TeleOp()
    {
        if(gamepad1.y && SlowModeTimer.milliseconds() > 250)
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

    void AutoBalance()
    {
        if(gamepad1.back && !autoBalancing)
        {
            slowModeDivisor = 1.0;
            autoBalancing = true;
            autoBalanceStable = false;
            autoBalanceTargetAngle = imu.getAngularOrientation().firstAngle;
        }

        if(autoBalancing)
        {
            // if the balance act has not been stable for half a second then try to balance
            if(!autoBalanceStable)
            {
                double autoBalanceTargetAngle = Math.toDegrees(Math.atan2((-imu.getAngularOrientation().secondAngle) * 10.0,
                        (-imu.getAngularOrientation().thirdAngle) * 10.0));
                double autoBalancePower = Math.min(Math.abs(((-imu.getAngularOrientation().thirdAngle * (1 / 30.0)))), 0.5);
                /*double autoBalanceTurnPower = Math.min(Math.abs((autoBalanceTargetAngle - imu.getAngularOrientation().firstAngle) * (1 / 90.0)), 0.5) *
                        Math.signum((autoBalanceTargetAngle - imu.getAngularOrientation().firstAngle));*/

                driveOmni45(autoBalanceTargetAngle, autoBalancePower, /*autoBalanceTurnPower*/ 0.0);
            }
            // if the balance is not stable then reset the timer
            if(Math.abs(imu.getAngularOrientation().secondAngle) > 2.0 || Math.abs(imu.getAngularOrientation().thirdAngle) > 2.0)
            {
                AutoBalanceTimeout.reset();
            }
            // if the balance is stable then check if the timer is above .5 seconds and if it is, then the balance is stable
            else if (AutoBalanceTimeout.milliseconds() > 500)
            {
                autoBalanceStable = true;
                autoBalancing = false;
            }
            // if the drive sticks move significantly, then cancel autobalancing and return to driver control
            if(Math.abs(gamepad1.right_stick_x) > 0.35 || Math.abs(gamepad1.right_stick_y) > 0.35 ||
                    Math.abs(gamepad1.left_stick_x) > 0.35 || Math.abs(gamepad1.left_stick_y) > 0.35)
            {
                autoBalancing = false;
                autoBalanceStable = false;
            }
        }

    }

    public void SendTelemetry()
    {
        telemetry.addData("GG Lift Ticks", motorGG.getCurrentPosition());
        telemetry.addData("GG distance from target", Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()));
        telemetry.addData("GG at position", Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) < 3);
        telemetry.addData("SlowMode", slowModeDivisor);
        telemetry.addData("FF Tics", Math.abs(motorFF.getTargetPosition() - motorFF.getCurrentPosition()));
        /*telemetry.addData("imu rot Y", imu.getAngularOrientation().firstAngle );
        telemetry.addData("imu rot X", imu.getAngularOrientation().secondAngle);
        telemetry.addData("imu rot Z", imu.getAngularOrientation().thirdAngle);
        telemetry.addData("Drive Angle", Math.toDegrees(Math.atan2((-imu.getAngularOrientation().secondAngle) * 10.0,
                (-imu.getAngularOrientation().thirdAngle) * 10.0)));
        telemetry.addData("Power", Math.min(Math.abs(((-imu.getAngularOrientation().thirdAngle * (1 / 25.0)))), 0.25));
        telemetry.addData("Turn Power", Math.min(Math.abs((autoBalanceTargetAngle - imu.getAngularOrientation().firstAngle) * (1 / 90.0)), 0.5) *
                Math.signum((autoBalanceTargetAngle - imu.getAngularOrientation().firstAngle)));
        telemetry.addData("autoBalancing", autoBalancing);*/
        //telemetry.addData("lift stage", liftStage);
        telemetry.update();
        idle();
    }

    void RunGGLift()
    {
        /* ===== GG STAGES ===== */

        if(!liftModeStateChange && (gamepad1.right_bumper || gamepad1.left_bumper))
        {
            if(gamepad1.right_bumper && motorGG.getCurrentPosition() < GGZero + 1850 - TOL)
            {
                motorGG.setTargetPosition(GGZero + 1900);
            }
            else if(gamepad1.right_bumper && motorGG.getCurrentPosition() >= GGZero + 1850 - TOL)
            {
                motorGG.setTargetPosition(GGZero + 3600);
            }
            else if(gamepad1.left_bumper && motorGG.getCurrentPosition() >= GGZero + 3550 - TOL)
            {
                motorGG.setTargetPosition(GGZero + 1900);
            }
            else if(gamepad1.left_bumper && motorGG.getCurrentPosition() <= GGZero + 3550 + TOL)
            {
                if(gamepad1.left_bumper && motorGG.getCurrentPosition() <= GGZero + 1850 + TOL)
                {
                    motorGG.setTargetPosition(GGZero + 40);
                }

                else
                {
                    motorGG.setTargetPosition(GGZero + 1850);
                }
            }
            motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 125.0), 1.0));
        }

        if (Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
        {
            motorGG.setPower(0.0);
        }

        /* ===== FLIPPY FLIP ===== */

        if (gamepad1.right_stick_button && GGFlipStage == 0)
        {
            GGStart = motorGG.getCurrentPosition();
            GGFlipStage = 1;
            GGFlipTimer.reset();
        }
        else if (GGFlipStage == 0)
        {
            motorFF.setTargetPosition(holdPosition);
            motorFF.setPower((motorFF.getTargetPosition() - motorFF.getCurrentPosition() * 400));
        }

        if(GGStart <= (GGZero + 300) && GGFlipStage == 1)
        {
            servoGGUL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
            servoGGUR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
            servoGGDL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
            servoGGDR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
            if(GGFlipTimer.milliseconds() > 125)
            {
                motorGG.setTargetPosition(GGZero + 300);
                motorGG.setPower(Math.signum(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) *
                        Math.min(Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 150.0), 1.0));
                if ((Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10))
                {
                    GGFlipStage++;
                    GGLifted = true;
                }
            }
        }
        else if (GGStart >= (GGZero + 300)  && GGFlipStage == 1)
            GGFlipStage ++;

        if (GGFlipStage == 2)
        {
            if(firstTime)
            {
                GGFlipTimer.reset();
                firstTime = false;
            }

            if (!GGFlipped)
            {
                motorFF.setTargetPosition(FFZero + 850);
                motorFF.setPower(Math.max(((motorFF.getTargetPosition() - motorFF.getCurrentPosition()) * (1 / 770.0)), 0.2));
                if ((Math.abs(motorFF.getTargetPosition() - motorFF.getCurrentPosition()) <= 5) || GGFlipTimer.milliseconds() > 1250)
                {
                    motorFF.setTargetPosition(motorFF.getCurrentPosition());
                    motorFF.setPower(0.0);
                    GGFlipped = true;
                    GGFlipStage = 3;
                    holdPosition = motorFF.getCurrentPosition();
                    firstTime = true;
                }
            }
            else
            {
                motorFF.setTargetPosition(FFZero - 100);
                motorFF.setPower(Math.min(((motorFF.getTargetPosition() - motorFF.getCurrentPosition()) * (1 / 770.0)), -0.2));
                if (Math.abs(motorFF.getTargetPosition() - motorFF.getCurrentPosition()) <= 5 || GGFlipTimer.milliseconds() > 1250)
                {
                    motorFF.setPower(0.0);
                    GGFlipped = false;
                    FFZero = motorFF.getCurrentPosition();
                    GGFlipStage = 3;
                    holdPosition = motorFF.getCurrentPosition();
                    firstTime = true;
                }
            }
        }
        if(GGStart < 3400 && GGFlipStage == 3)
        {
            motorGG.setTargetPosition(GGStart + 150);
            motorGG.setPower(Math.signum(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) *
                Math.min(Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 150.0), 1.0));
            if (Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
            {
                GGFlipStage = 0;
                GGLifted = false;
            }
        }
        else if (GGFlipStage == 3)
            GGFlipStage = 0;

        /* ====== FINE ADJUST ===== */

        if(gamepad1.right_trigger > 0.35 || gamepad1.left_trigger > 0.35)
        {
            while(gamepad1.right_trigger > 0.35)
            {
                motorGG.setTargetPosition(motorGG.getCurrentPosition() + 3000);
                liftModeStateChange = true;
                motorGG.setPower(0.25);
            }
            motorGG.setPower(0.0);
            liftModeStateChange = false;

            while(gamepad1.left_trigger > 0.35)
            {
                motorGG.setTargetPosition(motorGG.getCurrentPosition() - 3000);
                liftModeStateChange = true;
                motorGG.setPower(-0.25);
            }
            motorGG.setPower(0.0);
            liftModeStateChange = false;
        }
        if(gamepad1.left_stick_button)
        {
            GGZero = motorGG.getCurrentPosition();
        }

        /*if (motorGG.getCurrentPosition() > GGZero + 3750)
        {
            motorGG.setPower(0.0);
        }*/
        idle();
    }

    void RunGGClaws()
    {
        //Full close
        if(gamepad1.a)
        {
            servoGGUL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
            servoGGUR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
            servoGGDL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
            servoGGDR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
        }
        //half open
        else if(gamepad1.x)
        {
            servoGGUL.setPosition(GGServoPositions.LEFTHALFOPEN.val());
            servoGGUR.setPosition(GGServoPositions.RIGHTHALFOPEN.val());
            servoGGDL.setPosition(GGServoPositions.LEFTHALFOPEN.val());
            servoGGDR.setPosition(GGServoPositions.RIGHTHALFOPEN.val());
        }
        //full open
        else if(gamepad1.b)
        {
            servoGGUL.setPosition(GGServoPositions.LEFTFULLOPEN.val());
            servoGGUR.setPosition(GGServoPositions.RIGHTFULLOPEN.val());
            servoGGDL.setPosition(GGServoPositions.LEFTFULLOPEN.val());
            servoGGDR.setPosition(GGServoPositions.RIGHTFULLOPEN.val());
        }

        if(gamepad1.left_stick_button && gamepad1.right_stick_button && gamepad1.left_bumper)
        {
            GGZero = motorGG.getCurrentPosition() + 55;
        }

        if(gamepad1.dpad_down)
        {
            if(!GGFlipped)
            {
                servoGGDL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
                servoGGDR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
                bottomGGClosed = true;
            }
            else
            {
                servoGGUL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
                servoGGUR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
                topGGClosed = true;
            }
        }
        else if(gamepad1.dpad_up)
        {
            if(!GGFlipped)
            {
                servoGGUL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
                servoGGUR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
                topGGClosed = true;
            }
            else
            {
                servoGGDL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
                servoGGDR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
                bottomGGClosed = true;
            }
        }
        idle();
    }

    //region RR Code
    /*
    public void RunRR()
    {
        if(gamepad1.dpad_up && !RRMoving && !RRExtended)
        {
            motorRR.setTargetPosition(1680);
            motorRR.setPower(Math.signum(motorRR.getTargetPosition() - motorRR.getCurrentPosition()) *
                    Math.min(Math.abs(0.7 * ((motorRR.getTargetPosition() - motorRR.getCurrentPosition()) / 1680)), 0.1));
            RRExtended = true;
            RRMoving = true;
            RRAtPosition = false;
        }
        else if(gamepad1.dpad_down && !RRMoving && RRExtended)
        {
            motorRR.setTargetPosition(0);
            motorRR.setPower(Math.signum(motorRR.getTargetPosition() - motorRR.getCurrentPosition()) *
                    Math.min(Math.abs(0.7 * ((motorRR.getTargetPosition() - motorRR.getCurrentPosition()) / 1680)), 0.1));
            RRExtended = false;
            RRMoving = true;
            RRAtPosition = false;
        }
        if(motorIsAtTarget(motorRR))
        {
            RRAtPosition = true;
            motorRR.setTargetPosition(motorRR.getCurrentPosition());
            RRMoving = false;
        }
        if(RRAtPosition)
        {
            motorRR.setPower(Math.signum(motorRR.getTargetPosition() - motorRR.getCurrentPosition()) *
                    Math.min(Math.abs(0.2 * ((motorRR.getTargetPosition() - motorRR.getCurrentPosition()) / 1680)), 0.01));
        }

        if(gamepad1.dpad_up && RRExtended && !RRHandOpen && !HandMoving)
        {
            servoRRHand.setPosition(0.5); //OPEN
            RRHandOpen = !RRHandOpen;
            HandTimer.reset();
            HandMoving = true;
        }
        else if(gamepad1.dpad_up && RRExtended && RRHandOpen && !HandMoving)
        {
            servoRRHand.setPosition(0.8); //CLOSED
            RRHandOpen = !RRHandOpen;
            HandTimer.reset();
            HandMoving = true;
        }
        if(HandTimer.milliseconds() > 250 && HandMoving)
        {
            HandMoving = false;
        }


    }
    */
    //endregion
}
