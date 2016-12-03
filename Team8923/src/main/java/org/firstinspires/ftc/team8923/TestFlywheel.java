package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode is intended to test the flywheel to check for functionality, and to adjust any
 * numbers used in the code
 */
@TeleOp(name = "Flywheel Test", group = "Tests")
public class TestFlywheel extends LinearOpMode
{
    private DcMotor motorFlywheel;
    private Servo servoFlywheelAngler;
    private Servo servoBallPopper;

    private double power = 0.0;
    private double flywheelAngle = 0.5;
    private double popperAngle = 0.5;

    private static final double DELTA = 0.05;

    private boolean buttonWasPressed = false;

    // Used to calculate flywheel speed to set appropriate max speed
    private ElapsedTime timer = new ElapsedTime();
    private int lastEncoder = 0;
    private int deltaEncoder = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motorFlywheel = hardwareMap.dcMotor.get("motorFlywheel");
        motorFlywheel.setMaxSpeed(2700);
        motorFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        servoFlywheelAngler = hardwareMap.servo.get("servoFlywheelAngler");
        servoBallPopper = hardwareMap.servo.get("servoBallPopper");

        waitForStart();

        telemetry.log().add("Flywheel Angle: D-pad up and down");
        telemetry.log().add("Popper Angle: D-pad left and right");
        telemetry.log().add("Flywheel Power: y and a");

        while(opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                if(!buttonWasPressed)
                {
                    flywheelAngle += DELTA;
                    buttonWasPressed = true;
                }
            }
            else if(gamepad1.dpad_down)
            {
                if(!buttonWasPressed)
                {
                    flywheelAngle -= DELTA;
                    buttonWasPressed = true;
                }
            }
            else if(gamepad1.dpad_right)
            {
                if(!buttonWasPressed)
                {
                    popperAngle += DELTA;
                    buttonWasPressed = true;
                }
            }
            else if(gamepad1.dpad_left)
            {
                if(!buttonWasPressed)
                {
                    popperAngle -= DELTA;
                    buttonWasPressed = true;
                }
            }
            else if(gamepad1.y)
            {
                if(!buttonWasPressed)
                {
                    power += DELTA;
                    buttonWasPressed = true;
                }
            }
            else if(gamepad1.a)
            {
                if(!buttonWasPressed)
                {
                    power -= DELTA;
                    buttonWasPressed = true;
                }
            }
            else
                buttonWasPressed = false;

            motorFlywheel.setPower(power);
            servoFlywheelAngler.setPosition(flywheelAngle);
            servoBallPopper.setPosition(popperAngle);

            int currentEncoder = motorFlywheel.getCurrentPosition();
            if(currentEncoder - lastEncoder != 0)
                deltaEncoder = currentEncoder - lastEncoder;

            telemetry.addData("Power", power);
            telemetry.addData("Ticks Per Second", deltaEncoder / timer.seconds());
            telemetry.addData("Angle", flywheelAngle);
            telemetry.addData("Popper", popperAngle);
            telemetry.addData("Button Was Pressed", buttonWasPressed);

            timer.reset();
            lastEncoder = currentEncoder;

            telemetry.update();
            idle();
        }
    }
}
