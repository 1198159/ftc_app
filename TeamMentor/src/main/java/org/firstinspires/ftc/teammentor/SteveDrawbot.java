package org.firstinspires.ftc.teammentor;

import android.graphics.Point;

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

    servo 1 (s1) is connected to a base and rotates an arm of length a ("side a").
    servo 2 (s2) is at the end of that arm, and rotates another arm of length b ("side b").
    * is the pen


                                  *
                                  |
                                  | side b
                      side a      |
          servo1 ------------------ servo2

     [hmmm, we need a way to lift and lower the pen...]

     Let alpha be the angle of servo1 relative to a baseline
     and let beta be the angle of servo2 relative to side a (the bar between servo1 and servo2).

     We can calculate the required angles using trig. (more about that below)

     We will use vuforia to "look" at a piece of white paper that has a drawing in black ink.
     We will obtain a bitmap from vuforia, scan the lines of the bitmap for non-white pixels,
     and have the drawbot ink those locations on a destination piece of paper.

     To "draw" the image, we need to convert the desired coordinates for the pen into
     the two angles (i.e., servo positions for servo1 and servo2).

     There is a handy article here that explains the math.
     https://math.stackexchange.com/questions/1423467/calculating-angles-neccessary-to-reach-a-position-on-a-2d-plane-for-two-robot-ar


     */


    Servo servo1, servo2;

    Double len_a = 10.0; //length of bar from servo1 to servo2
    Double len_b = 5.0;  //length of bar from servo2 to pen
    double len_c = 0.0;  //length from servo1 to pen. This will be calculated as needed for each point of the drawing.

    //     Let alpha be the angle of servo servo1 relative to a baseline
    //     and let beta be the angle of servo servo2 relative to side a (the bar between servo1 and servo2).
    public double angleAlpha;
    public double angleBeta;

    double DEFAULT_ANGLE_ALPHA = 0.0;
    double DEFAULT_ANGLE_BETA = 0.0;

    // some variables to turn the pen on and off
    boolean PEN_DRAW = true;
    boolean PEN_NOT_DRAW = false;
    boolean penState = PEN_NOT_DRAW;


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

    private void initializeServos()
    {
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");

        angleAlpha = DEFAULT_ANGLE_ALPHA;
        angleBeta = DEFAULT_ANGLE_BETA;

        updateServoPositions();
    }


    private void initializeRobot()
    {
        initializeServos();

        // Set up telemetry data
        //configureDashboard();
    }


    private void updateServoPositions()
    {
        servo1.setPosition(angleAlpha);
        servo2.setPosition(angleBeta);
    }

    //put the pen in a state in which it does not draw lines
    private void setPenState(boolean desiredPenState)
    {
        penState = desiredPenState;
    }

    private void calculateServoAnglesForPoint(double x, double y)
    {
        //cribbing heavily from:
        // https://math.stackexchange.com/questions/1423467/calculating-angles-neccessary-to-reach-a-position-on-a-2d-plane-for-two-robot-ar

        //c is the distance from the origin to the target location (x,y).
        //The formula for the distance between two points is  Math.sqrt( (x2 - x1)^2 + (y2 - y1)^2 )
        //Luckily, the origin is conveniently (0,0), which simplifies the formula for us.
        double c = Math.sqrt( (x * x) + (y * y) );

        angleBeta = Math.acos(
                        (len_a * len_a) + (len_b * len_b) - (c * c) /
                                (2 * len_a * len_b)
        );


        double g = Math.atan2(x, y);
        double t = Math.acos(
                        (len_a * len_a) + (c * c) - (len_b * len_b)  /
                               (2 * len_a * c)
        );

        angleAlpha = g + t;

    }

    private void moveTo(double x, double y)
    {
        setPenState(PEN_NOT_DRAW);

        calculateServoAnglesForPoint(x, y);

        updateServoPositions();
    }

    private void drawTo(double x, double y)
    {
        setPenState(PEN_DRAW);

        calculateServoAnglesForPoint(x, y);

        updateServoPositions();
    }



    private void configureDashboard()
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

    private String formatNumber(double d)
    {
        return String.format("%.2f", d);
    }
    private String formatNumberEightDigits(double d)
    {
        return String.format("%.8f", d);
    }
}
