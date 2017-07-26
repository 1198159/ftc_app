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
//@TeleOp(name="BalanceBot", group = "Steve")
// @Disabled
abstract public class SteveBalancebotMaster extends LinearOpMode
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
     *  NEVEREST 20's
     *  0.01 was not enough correction?
     *  0.05 looks like not enough correction; the robot keeps leaning and then starts rocking back and forth quickly
     *  0.1 rocks back and forth even more violently
     *  0.02 seems "pretty close" but still too weak
     *  0.03 seems "almost there" but still a bit too weak
     *
     *
     *  NEVEREST 40's
     *
    */

    //These PID constants assume Neverest 40's.
    public double P_CONSTANT = 0.08;
    public double I_CONSTANT = 0.0000;
    public double D_CONSTANT = 0.0000;

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
    }

}
