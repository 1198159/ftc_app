package org.firstinspires.ftc.teammentor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;


/**
 * Program used to control a self-balancing robot on two wheels.
 */
@TeleOp(name="SteveBalanceBot", group = "Swerve")
// @Disabled
public class SteveBalanceBot extends LinearOpMode
{
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    //imu sensor object
    BNO055IMU imu = null;

    // State used for updating telemetry
    Orientation angles;
    //Acceleration gravity;


    //The IMU emits heading ("first angle"), roll ("second angle"), and pitch ("third angle").
    //On my robot, roll determines whether the robot is balanced.
    double targetRoll = 0.0;

    double currentRoll = 0.0;

    FilterPID filterPID = null;


    //notes about P constant
    /*
     *  0.01 was not enough correction?
     *  0.05 looks like not enough correction; the robot keeps leaning and then starts rocking back and forth quickly
     *  0.1 rocks back and forth even more violently
     *  0.02 seems "pretty close" but still too weak
     *  0.03 seems "almost there" but still a bit too weak
     *
    */

    private double P_CONSTANT = 0.0321;
    private double I_CONSTANT = 0.0000;
    private double D_CONSTANT = 0.001;

    @Override public void runOpMode() throws InterruptedException
    {
        double error = 0;
        double motorSpeed = 0.0;

        // Initialize hardware and other important things
        initializeRobot();

        // Wait until start button has been pressed
        waitForStart();

        //wait for IMU to start running
        while (imu.getSystemStatus() != BNO055IMU.SystemStatus.RUNNING_FUSION)
        {
            telemetry.addData("init", "starting imu...");
            telemetry.addData("state", imu.getSystemStatus());
            idle();
        }

        // Main loop
        while(opModeIsActive())
        {

            //some gamepad controls that may help me tune the PID constants
            if (gamepad1.a)
            {
                while (gamepad1.a) {}
                D_CONSTANT += 0.001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.b)
            {
                while (gamepad1.b) {}
                D_CONSTANT -= 0.001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.x)
            {
                while (gamepad1.x) {}
                P_CONSTANT += 0.001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.y)
            {
                while (gamepad1.y) {}
                P_CONSTANT -= 0.001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.dpad_up)
            {
                while (gamepad1.dpad_up) {}
                P_CONSTANT += 0.0001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.dpad_down)
            {
                while (gamepad1.dpad_down) {}
                P_CONSTANT -= 0.0001;
                filterPID.updateFilterConstants(P_CONSTANT, I_CONSTANT, D_CONSTANT);
            }
            if (gamepad1.left_bumper)
            {
                telemetry.addData("KP", P_CONSTANT);
                telemetry.addData("KI", I_CONSTANT);
                telemetry.addData("KD", D_CONSTANT);
            }



            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //currentRoll = AngleUnit.DEGREES.normalize(degrees)
            currentRoll = AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.secondAngle));
            //telemetry.addData("cur", currentRoll);

            error = targetRoll - currentRoll;

            motorSpeed = filterPID.getFilteredValue(error);
            //telemetry.addData("speed", motorSpeed);

            motorLeft.setPower(motorSpeed);
            motorRight.setPower(motorSpeed);

            telemetry.update();
            idle();
        }
    }


    public void initializeRobot()
    {
        //Connect our motor member variables to the hardware motors
        motorLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotor.class, "motorRight");
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        filterPID = new FilterPID(this, P_CONSTANT, I_CONSTANT, D_CONSTANT);

        // Set up telemetry data
        configureDashboard();
    }

    public void configureDashboard()
    {
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        //this is no longer needed since I get this as part of my main loop
        /*
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });
        */

        /*
        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });
         */

        telemetry.addData("currRoll", new Func<String>() {
            @Override public String value() {
                return formatDegrees(currentRoll);
            }
        });

        telemetry.addLine()
                .addData("P", new Func<String>() {
                    @Override public String value() {
                return formatNumber(P_CONSTANT);
                }
                })
                .addData("I", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(I_CONSTANT);
                    }
                })
                .addData("D", new Func<String>() {
                    @Override public String value() {
                        return formatNumber(D_CONSTANT);
                    }
                });


    }

    public String formatNumber(double d)
    {
        return String.format("%.4f", d);
    }
    public String formatNumberEightDigits(double d)
    {
        return String.format("%.8f", d);
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
}
