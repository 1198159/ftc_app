package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
    A simple opmode that shows how to use a digital input/output pin.
    This could be used to control a relay, an LED, etc. 
 */


@TeleOp(name = "Digital Pin Test", group = "Tests")
@Disabled
public class DigitalPinTest extends LinearOpMode
{

    DigitalChannel myRelay;

    public void runOpMode() throws InterruptedException
    {

        myRelay = hardwareMap.digitalChannel.get("relay");
        myRelay.setMode(DigitalChannel.Mode.OUTPUT);
        myRelay.setState(false);

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                myRelay.setState(true);
                telemetry.log().add("relay turned on");
            }
            if(gamepad1.b)
            {
                myRelay.setState(false);
                telemetry.log().add("relay turned off");
            }

            telemetry.addData("Time", System.currentTimeMillis());
            telemetry.update();
            idle();
        }

    }
}
