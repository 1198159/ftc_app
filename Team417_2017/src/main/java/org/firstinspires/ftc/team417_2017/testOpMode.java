package org.firstinspires.ftc.team417_2017;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * A test to confirm that we can compile code in each new team project
 */
//@TeleOp(name="test 417_2017", group = "Swerve")
// @Disabled
public class testOpMode extends LinearOpMode
{
    DcMotor motor; // We are testing AndyMark NeverRest 20s, 40s, 60s, 3.7s, Matrix, and REV Core Hex motors (three of each) plugged into port 1.

    @Override public void runOpMode() throws InterruptedException
    {
        // Wait until start button has been pressed
        waitForStart();

        motor = hardwareMap.dcMotor.get("motor");
        //motor.setDirection(DcMotor.Direction.REVERSE); // reverse the motor
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // allows the motor to slow down without brakes
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this mode simply inputs power, so no PID
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // this mode will do its best to run at a targeted velocity

        // Tell the user that the motor is done initializing
        telemetry.addData(">", "Motor done initializing");
        telemetry.update();
        motor.setPower(-0.5);
        sleep(30000);
        motor.setPower(0.0);
        idle();

        /*
        // Main loop
        while(opModeIsActive())
        {

        }
        */
    }

}
