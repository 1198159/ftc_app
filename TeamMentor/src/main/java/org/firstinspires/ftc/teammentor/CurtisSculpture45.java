package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * This file contains an opmode that tests whether an encoder is working.
 */

@TeleOp(name="45", group="Curtis")  // @Autonomous(...) is the other common choice
//@Disabled
public class CurtisSculpture45 extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor motor = null;
    //DcMotor rightMotor = null;

    @Override
    public void runOpMode() {

        motor  = hardwareMap.dcMotor.get("motor");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        motor.setPower(0.45);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            idle();

        }
    }
}
