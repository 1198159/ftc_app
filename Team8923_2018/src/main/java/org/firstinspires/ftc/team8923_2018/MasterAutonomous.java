package org.firstinspires.ftc.team8923_2018;

import com.qualcomm.robotcore.util.ElapsedTime;

abstract class MasterAutonomous extends Master
{
    private ElapsedTime runtime = new ElapsedTime();

    double newTargetFL;
    double newTargetFR;
    double newTargetBL;
    double newTargetBR;

    double errorFL;
    double errorFR;
    double errorBL;
    double errorBR;
    double avgDistError;
    double TOL = 100.0;

    boolean doneSettingUp = false;

    Alliance alliance = Alliance.BLUE;
    StartLocations startLocation = StartLocations.DEPOT;

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
    }

    public void moveLift (int ticks)
    {
        motorLift.setTargetPosition(motorLift.getCurrentPosition() + ticks);
        motorLift.setPower((motorLift.getTargetPosition() - motorLift.getCurrentPosition()) * (1 / 1000.0));
        idle();;
    }
    public void stopLift()
    {
        motorLift.setPower(0.0);
        idle();
    }

    public void driveMecanumAuto (double driveAngle, double drivePower, double TurnPower, double moveMM, int timeout)
    {
        runtime.reset();
        newTargetFL = motorFL.getTargetPosition() + moveMM / MM_PER_TICK;
        newTargetFR = motorFR.getTargetPosition() + moveMM / MM_PER_TICK;
        newTargetBL = motorBL.getTargetPosition() + moveMM / MM_PER_TICK;
        newTargetBR = motorBR.getTargetPosition() + moveMM / MM_PER_TICK;
        do
        {
            driveMecanum(driveAngle, drivePower, TurnPower);
            errorFL = newTargetFL - motorFL.getCurrentPosition();
            errorFR = newTargetFR - motorFR.getCurrentPosition();
            errorBL = newTargetBL - motorBL.getCurrentPosition();
            errorBR = newTargetBR - motorBR.getCurrentPosition();
            avgDistError = (errorFL + errorFR + errorBL + errorBR) / 4.0;
        }
        while(opModeIsActive() && (runtime.seconds() < timeout) &&
        (avgDistError < TOL));
        stopDriving();
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
    }

    public void stopDriving ()
    {
        motorFL.setPower(0.0);
        motorFR.setPower(0.0);
        motorBL.setPower(0.0);
        motorBR.setPower(0.0);
    }

    
}
