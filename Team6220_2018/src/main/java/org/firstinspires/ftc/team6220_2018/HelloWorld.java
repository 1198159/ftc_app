package org.firstinspires.ftc.team6220_2018;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;

/**
 * This is a simple "hello world" opmode
 *
 */

@TeleOp(name="HelloWorld", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class HelloWorld extends LinearOpMode {

    @Override
    public void runOpMode() {

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("greeting", "hello world!");

            while (opModeIsActive()) {
                telemetry.update();
                idle();
            }

        }
    }
}
