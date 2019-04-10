package org.firstinspires.ftc.team417_2018;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

@Autonomous(name = "Test OpMode")
//@Disabled
public class TestOpMode extends MasterAutonomous
{
    public void runOpMode() throws InterruptedException
    {

        //autoInitializeRobot();
        InitializeDetection();

        OpenCV_detector.setThreshold(threshold);
        telemetry.addData("threshold", threshold);

        if (isPosCrater) telemetry.addData("Alliance: ", "CRATER");
        else telemetry.addData("Alliance: ", "DEPOT");

        telemetry.addData("Gold",
                String.format(Locale.getDefault(), "(%d, %d)", (OpenCV_detector.getGoldRect().x + OpenCV_detector.getGoldRect().width / 2), (OpenCV_detector.getGoldRect().y + OpenCV_detector.getGoldRect().height / 2)));

        telemetry.addData("Init:", "done");
        telemetry.update();
        waitForStart();

        //landNew(2000, 2900);
        //autoExtendSlides();
        locateGold();
        //reset();
    }
}
