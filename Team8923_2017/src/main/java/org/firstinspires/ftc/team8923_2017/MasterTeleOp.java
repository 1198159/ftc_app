package org.firstinspires.ftc.team8923_2017;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Holds all code necessary to run the robot in driver controlled mode
 */

public abstract class MasterTeleOp extends Master
{
    private boolean liftModeStateChange = false;
    private boolean GGFlipped = false;
    private boolean GGLifted = false;
    private boolean FlipTimerNotJustReset = true; // used to reset GGFlipTimer on the first time through the statement
    private boolean autoBalanceStable = false;
    private boolean GGSafetyEscapePositionSet = false;
    private boolean FFFudgeTimerReset = false;
    boolean autoBalancing = false;

    private int GGStart;
    private int GGFlipStage = 0;
    private int holdPosition;
    private double TriggerIntToBoolVal = 0.10;

    private ElapsedTime SlowModeTimer = new ElapsedTime();
    private ElapsedTime GGFlipTimer = new ElapsedTime();
    private ElapsedTime AutoBalanceTimeout = new ElapsedTime();
    private ElapsedTime GGClawReset = new ElapsedTime();
    private ElapsedTime FFFudgeTimer = new ElapsedTime();
    private ElapsedTime SSATimeout = new ElapsedTime();

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
        if(gamepad1.dpad_right && !autoBalancing)
        {
            slowModeDivisor = 1.0;
            autoBalancing = true;
            autoBalanceStable = false;
            //double autoBalanceTargetAngle = imu.getAngularOrientation().firstAngle;
        }

        if(autoBalancing)
        {
            // if the balance act has not been stable for half a second then try to balance
            if(!autoBalanceStable)
            {
                double autoBalanceTargetAngle = Math.toDegrees(Math.atan2((-imu.getAngularOrientation().secondAngle) * 10.0,
                        (-imu.getAngularOrientation().thirdAngle) * 10.0));
                double autoBalancePower = Math.max(Math.min(Math.abs(((-imu.getAngularOrientation().thirdAngle * (1 / 30.0)))), 0.6), 0.17);
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
        telemetry.addData("GG Lift Ticks", motorGG.getCurrentPosition() - GGZero);
        telemetry.addData("FF Tics", FFZero - motorFF.getCurrentPosition());
        telemetry.addData("FF distance to target", Math.abs(motorFF.getTargetPosition() - motorFF.getCurrentPosition()));
        telemetry.addData("GG distance from target", Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()));
        telemetry.addData("GG at position", Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) < 3);
        telemetry.addData("SlowMode", slowModeDivisor);
        telemetry.addData("Dpad_Up", gamepad1.dpad_up);
        telemetry.addData("Dpad_Down", gamepad1.dpad_down);
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
                motorGG.setTargetPosition(GGZero + 3700);
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
            motorGG.setPower(Math.max((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 75.0), 1.0));
        }

        if (Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
        {
            motorGG.setPower(0.0);
        }

        /* ===== FLIPPY FLIP ===== */

        //if right stick button pressed, enter FF flip state machine
        if (gamepad1.right_stick_button && GGFlipStage == 0)
        {
            GGStart = motorGG.getCurrentPosition();
            GGFlipStage = 1;
            GGFlipTimer.reset();
        }
        //FF actively holds position when it's not doing anything
        else if (GGFlipStage == 0)
        {
            motorFF.setTargetPosition(holdPosition);
            motorFF.setPower((motorFF.getTargetPosition() - motorFF.getCurrentPosition()) * (1 / 100.0));
        }

        //move GG up if it starts too low to flip without a jam
        if(GGStart <= (GGZero + 600) && GGFlipStage == 1)
        {
            //close the servos to pick up glyphs that might get left behind and prevent
            //damage to servo arms that could get caught on the ground
            servoGGL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
            servoGGR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
            //wait for servos to close
            if(GGFlipTimer.milliseconds() > 125)
            {
                //raise GG to a position that it is safe for the FF to rotate
                motorGG.setTargetPosition(GGZero + 600);
                motorGG.setPower(Math.signum(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) *
                        Math.min(Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 50.0), 1.0));
                if ((Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10))
                {
                    GGFlipStage++;
                    GGLifted = true;
                }
            }
        }
        //otherwise just move on
        else if (GGStart >= (GGZero + 600)  && GGFlipStage == 1)
            GGFlipStage ++;

        //Flip the FF close to the hardware stop using encoders
        if (GGFlipStage == 2)
        {
            //reset timer first time through
            if(FlipTimerNotJustReset)
            {
                GGFlipTimer.reset();
                FlipTimerNotJustReset = false;
            }

            //flip the GG if it isn't
            if (!GGFlipped)
            {
                motorFF.setTargetPosition(FFZero - 725);
                motorFF.setPower(Math.min(((motorFF.getTargetPosition() - motorFF.getCurrentPosition()) * (1 / 770.0)), -0.3));
                if ((Math.abs(motorFF.getTargetPosition() - motorFF.getCurrentPosition()) <= 5))
                {
                    motorFF.setPower(0.0);
                    FlipTimerNotJustReset = true;
                    GGFlipStage = 3;
                }
                // FF jam safety feature
                else if(GGFlipTimer.milliseconds() > 2000)
                {
                    motorFF.setPower(0.0);

                    if(motorGG.getCurrentPosition() < 3450 && !GGSafetyEscapePositionSet)
                    {
                        motorGG.setTargetPosition(motorGG.getCurrentPosition() + 450);
                        GGSafetyEscapePositionSet = true;
                    }
                    else if(motorGG.getCurrentPosition() > 3450 && !GGSafetyEscapePositionSet)
                    {
                        motorGG.setTargetPosition(motorGG.getCurrentPosition());
                        GGSafetyEscapePositionSet = true;
                    }

                    motorGG.setPower((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 175.0));

                    if(Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
                    {
                        motorGG.setPower(0.0);
                        GGFlipTimer.reset();
                        GGSafetyEscapePositionSet = false;
                    }
                }
            }
            //un-flip the GG otherwise
            else
            {
                motorFF.setTargetPosition(FFZero - 25);
                motorFF.setPower(Math.max(((motorFF.getTargetPosition() - motorFF.getCurrentPosition()) * (1 / 770.0)), 0.3));

                if (Math.abs(motorFF.getTargetPosition() - motorFF.getCurrentPosition()) <= 5)
                {
                    motorFF.setPower(0.0);
                    FlipTimerNotJustReset = true;
                    GGFlipStage = 3;
                }
                // FF jam safety feature
                else if(GGFlipTimer.milliseconds() > 2000)
                {
                    motorFF.setPower(0.0);

                    if(motorGG.getCurrentPosition() < 3450 && !GGSafetyEscapePositionSet)
                    {
                        motorGG.setTargetPosition(motorGG.getCurrentPosition() + 450);
                        GGSafetyEscapePositionSet = true;
                    }
                    else if(motorGG.getCurrentPosition() > 3450 && !GGSafetyEscapePositionSet)
                    {
                        motorGG.setTargetPosition(motorGG.getCurrentPosition());
                        GGSafetyEscapePositionSet = true;
                    }

                    motorGG.setPower((motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 175.0));

                    if(Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
                    {
                        motorGG.setPower(0.0);
                        GGFlipTimer.reset();
                        GGSafetyEscapePositionSet = false;
                    }
                }
            }
        }

        //make sure FF is fully on the hardware stop
        if(GGFlipStage == 3)
        {
            if(!FFFudgeTimerReset)
            {
                FFFudgeTimer.reset();
                FFFudgeTimerReset = true;
            }
            if(FFFudgeTimer.milliseconds() < 125)
            {
                if(!GGFlipped)
                {
                    motorFF.setTargetPosition(FFZero - 1000);
                    motorFF.setPower(-1.0);
                }
                else
                {
                    motorFF.setTargetPosition(FFZero + 200);
                    motorFF.setPower(1.0);
                }
            }
            else
            {
                FFFudgeTimerReset = false;
                FFFudgeTimer.reset();
                motorFF.setPower(0.0);
                if(!GGFlipped)
                    holdPosition = motorFF.getCurrentPosition() + 5;
                else
                    holdPosition = motorFF.getCurrentPosition() - 5;
                GGFlipStage = 4;
                GGFlipped = !GGFlipped;
            }
        }

        //move the GG to the elevated end value if it started on the first stage
        if(GGLifted && GGFlipStage == 4)
        {
            motorGG.setTargetPosition(GGZero + 250);
            motorGG.setPower(Math.signum(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) *
                Math.min(Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) * (1 / 150.0), 1.0));
            if (Math.abs(motorGG.getTargetPosition() - motorGG.getCurrentPosition()) <= 10)
            {
                GGFlipStage = 0;
                GGLifted = false;
            }
        }
        //otherwise go back to the beginning
        else if (GGFlipStage == 4)
            GGFlipStage = 0;

        /* ====== FINE ADJUST ===== */

        if(gamepad1.right_trigger > TriggerIntToBoolVal || gamepad1.left_trigger > TriggerIntToBoolVal)
        {
            if(gamepad1.right_trigger > TriggerIntToBoolVal)
            {
                motorGG.setTargetPosition(GGZero + 5000);
                liftModeStateChange = true;
                motorGG.setPower(0.60);
            }
            else if(gamepad1.left_trigger > TriggerIntToBoolVal)
            {
                motorGG.setTargetPosition(GGZero - 1000);
                liftModeStateChange = true;
                motorGG.setPower(-0.60);
            }
        }
        else if (liftModeStateChange)
        {
            motorGG.setTargetPosition(motorGG.getCurrentPosition());
            motorGG.setPower(0.0);
            liftModeStateChange = false;
        }
        if(gamepad1.left_stick_button)
        {
            GGZero = motorGG.getCurrentPosition();
        }

        idle();
    }

    void RunGGClaws()
    {
        //Full close
        if(gamepad1.a)
        {
            servoGGL.setPosition(GGServoPositions.LEFTFULLCLOSED.val());
            servoGGR.setPosition(GGServoPositions.RIGHTFULLCLOSED.val());
        }
        //half open
        else if(gamepad1.x)
        {
            servoGGL.setPosition(GGServoPositions.LEFTHALFOPEN.val());
            servoGGR.setPosition(GGServoPositions.RIGHTHALFOPEN.val());
        }
        //full open
        else if(gamepad1.b)
        {
            servoGGL.setPosition(GGServoPositions.LEFTFULLOPEN.val());
            servoGGR.setPosition(GGServoPositions.RIGHTFULLOPEN.val());
        }

        if(gamepad1.left_stick_button && gamepad1.right_stick_button && gamepad1.left_bumper)
        {
            GGZero = motorGG.getCurrentPosition() + 55;
        }

        idle();
    }

    void RunVortex()
    {
        if(!(Math.abs(gamepad2.left_stick_y) < 0.0))
            SSATimeout.reset();

        if(!(Math.abs(gamepad2.left_stick_y) == 0.0) || gamepad2.right_bumper || gamepad2.right_trigger > TriggerIntToBoolVal
                || gamepad2.left_bumper || gamepad2.left_trigger > TriggerIntToBoolVal) // to decrease compute time (possibly)
        {
            if (SSATimeout.milliseconds() < 1.0) // run time of the arms before they throttle down
            {
                if (gamepad2.left_stick_y > 0.0)
                {
                    servoSSAL.setPower(1.0);
                    servoSSAR.setPower(-1.0);
                } else if (gamepad2.left_stick_y < 0.0)
                {
                    servoSSAL.setPower(-1.0);
                    servoSSAR.setPower(1.0);
                }
            } else
            {
                if (gamepad2.left_stick_y > 0.0)
                {
                    servoSSAL.setPower(0.2);
                    servoSSAR.setPower(-0.2);
                } else if (gamepad2.left_stick_y < 0.0)
                {
                    servoSSAL.setPower(-0.2);
                    servoSSAR.setPower(0.2);
                }
            }

            if (gamepad2.left_trigger > TriggerIntToBoolVal)
            {
                motorSSL.setPower(-0.5);
                servoSSAL.setPower(-0.3);
            }
            else if (gamepad2.left_bumper)
            {
                motorSSL.setPower(0.5);
                servoSSAL.setPower(-0.3);
            }
            if (gamepad2.right_trigger > TriggerIntToBoolVal)
            {
                motorSSR.setPower(0.5);
                servoSSAR.setPower(0.3);
            }
            else if (gamepad2.right_bumper)
            {
                motorSSR.setPower(-0.5);
                servoSSAR.setPower(0.3);
            }
        }
        else
        {
            motorSSL.setPower(0.0);
            motorSSR.setPower(0.0);
            servoSSAL.setPower(0.0);
            servoSSAR.setPower(0.0);
        }
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
