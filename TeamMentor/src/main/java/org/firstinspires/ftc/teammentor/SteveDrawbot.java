package org.firstinspires.ftc.teammentor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;


/**
 * Program used to control a robot that draws what it sees through the robot phone camera.
 */
@TeleOp(name="Drawbot", group = "Swerve")
// @Disabled
public class SteveDrawbot extends LinearOpMode
{

    /*

    Our drawing bot has the following hardware construction:

    servo 1 (s1) is connected to the base and rotates an arm of length a.
    servo 2 (s2) is at the end of that arm, and rotates another arm of length b.
    * is the pen

                    <- a ->
              s2 ------------------ s1
              |
          b   |
              |
              *

         [hmmm, we need a way to lift and lower the pen...]
     Let alpha be the angle of servo s1 and let beta be the angle of servo s2.

     We can calculate the required angles using trig. (more about that below)

     We will use vuforia to "look" at a piece of white paper that has a drawing in black ink.
     We will obtain a bitmap from vuforia, scan the lines of the bitmap for non-white pixels,
     and have the drawbot ink those locations on a destination piece of paper.

     To "draw" the image, we need to convert the desired coordinates for the pen into
     the two angles (i.e., servo positions for s1 and s2).

     There is a handy article here that explains the math.
     https://math.stackexchange.com/questions/1423467/calculating-angles-neccessary-to-reach-a-position-on-a-2d-plane-for-two-robot-ar


     */


    Servo servo1, servo2;

    Double len_a = 10.0; //length of bar from s1 to s2, in inches
    Double len_b = 5.0;  //length of bar from s2 to pen, in inches
    double len_c = 0.0;  //length from s1 to pen, in inches. This will be calculated as needed for each point of the drawing.

    Double angle_alpha = 0.0; //angle for servo s1
    Double angle_beta = 0.0;  //angle for servo s2


    @Override public void runOpMode() throws InterruptedException
    {
        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        // Main loop
        while(opModeIsActive())
        {
            //nothing yet, this is just a stub


            telemetry.update();
            idle();
        }
    }


    public void initializeRobot()
    {

        // Set up telemetry data
        //configureDashboard();
    }

    public void configureDashboard()
    {
        /*
        telemetry.addLine()
                .addData("Power | 1: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor1.getPower());
                    }
                })
                .addData("2: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor2.getPower());
                    }
                })
                .addData("3: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor3.getPower());
                    }
                })
                .addData("4: ", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(motor4.getPower());
                    }
                });
                */
    }

    public String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
    public String formatNumberEightDigits(double d)
    {
        return String.format("%.8f", d);
    }
}
