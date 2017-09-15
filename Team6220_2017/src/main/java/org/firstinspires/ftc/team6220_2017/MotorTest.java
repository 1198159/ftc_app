package org.firstinspires.ftc.team6220_2017;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;

/**
 * Created by Cole Welch on 8/29/2017.
 */
@TeleOp(name="MotorTest", group = "6220")
public class MotorTest extends MasterOpMode
{
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    public void initializeRobot()
    {
        // Initialize motors to be the hardware motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        // We're not using encoders, so tell the motor controller
        motorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // The motors will run in opposite directions, so flip one
        //THIS IS SET UP FOR TANK MODE WITH OUR CURRENT DRIVABOTS
        //DON'T CHANGE IT!
        motorRight.setDirection(DcMotor.Direction.REVERSE); //DO NOT change without talking to Heidi first!!!
    }

    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        //move motors constantly
        motorLeft.setPower(0.5);
        motorRight.setPower(0.5);

        // Main loop
        while(opModeIsActive())
        {
            //button inputs
            if (gamepad1.a)
            {
                motorLeft.setPower(0.0);
                motorRight.setPower(0.0);
            }
           /*if (gamepad1.b)
            {
                motorLeft.setPower(0.5);
                motorRight.setPower(0.5);
            }
            if (gamepad1.x)
            {
                motorLeft.setPower(1.0);
                motorRight.setPower(1.0);
            }*/
            motorLeft.setPower(gamepad1.left_stick_x);
            motorRight.setPower(gamepad1.left_stick_x);

            //telemetry data
            telemetry.addData("useful data", motorLeft.getPower());
            telemetry.update();
            idle();
        }
    }
}
