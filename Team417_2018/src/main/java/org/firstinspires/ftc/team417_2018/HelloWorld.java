package org.firstinspires.ftc.team417_2018;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.corningrobotics.enderbots.endercv.ActivityViewDisplay;

import java.util.Locale;
import java.util.Random;


@TeleOp(name="TestServos", group="Swerve")  // @Autonomous(...) is the other common choice
//@Disabled
public class HelloWorld extends MasterTeleOp {
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                vex1.setPower(1);
                sleep(2000);
                vex1.setPower(0.5);
            }
        }
    }
}
