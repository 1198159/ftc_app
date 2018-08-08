package org.firstinspires.ftc.teamswerve; // This software belongs in the Team Swerve Folder
// Below are all of the imports from other classes including DcMotor, DigitalChannel, and ElapsedTime.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Const;

// The name of this test is "DynamomterTests", and it belongs to the group "Swerve".
@TeleOp(name="DynamomterTests", group = "Swerve")
// @Disabled // when NOT disabled, "DynamomterTests" will show up on the select OpMode screen, under "TeleOp"
public class DynamomterProject extends LinearOpMode // "DynamometerProject" is a subclass of the base class "LinearOpMode".
{
    DcMotor motor; // We are testing AndyMark NeverRest 20s, 40s, 60s, 3.7s, Matrix, and REV Core Hex motors (three of each) plugged into port 1.
    DcMotorEx motorEx = null;
    DigitalChannel relay; // Relay circuit, plugged into port 0 of the REV module
    INA219 ina; // INA219 Current Sensor, plugged into port 1 of the REV module.  Here's the link to the data sheet: https://cdn-shop.adafruit.com/datasheets/ina219.pdf

    double motorPower = 0.0; // used as a placeholder value for incremented motor power the method "RampUpMotor" below
    double ticksPerSec = 0.0;
    int waitTime = 5; // a value (in milliseconds) used for time between each motor power increment in the method "RampUpMotor" below, and in timed samples
    double motorSetPower = 0.4; // the max power the motor power will increment to in the method "RampUpMotor" below
    int numSamples = 4000; // the maximum number of data samples the arrays for time, encoder counts, current, and voltage will store


    private ElapsedTime runtime = new ElapsedTime(); // used for timer that starts as soon as the play button is pushed
    private ElapsedTime sampleTime = new ElapsedTime(); // used as a separate timer in the timed sample loop, "TimedSamplingTest"
    int index = 0; // used to keep track of arrays
    // declare parallel arrays, made parallel by the same index
    double[] time = new double[numSamples]; // logs time every time a new encoder value is reached
    int[] motorPos = new int[numSamples]; // logs encoder counts
    double[] busVoltage = new double[numSamples]; // logs voltage every time a new encoder value is reached
    double[] shuntCurrent = new double[numSamples]; // logs shunt current every time a new encoder value is reached
    // second set for logging after the relay is turned off, named the same as the four arrays above,
    // just with a "2" at the end to indicate they're used after the relay is turned off
    double[] time2 = new double[numSamples];
    int[] motorPos2 = new int[numSamples];
    double[] busVoltage2 = new double[numSamples];
    double[] shuntCurrent2 = new double[numSamples];

    boolean isAPressed;
    boolean isMotorOn;
    boolean isDpadUpPushed;
    boolean isDpadDownPushed;


    /*
    The following two methods follow the same concept as the two methods above.  They both log data
    in specific time increments instead of for every encoder count.  Different from the two methods
    above, the two methods below keep the time with a separate timer called "sampleTime".  During
    the log loop, the sample timer "sampleTime" is checked to see if it is greater than the
    specified "waitTime".  If it is, then the time (from the separate "runtime" timer, motor
    position, bus voltage, and shunt current will be logged.  After all of the four data points have
    been logged, the index is incremented for the new data point and the sample timer is reset.
     */
    public void TimedSamplingTest()
    {
        index = 0; // reset the index value to 0
        sampleTime.reset(); // reset the sample timer
        while (runtime.milliseconds() < 30000) // while the motor hasn't been running for 30 seconds...
        {
            // if the sample time is less than the intended time between each sample...
            if (sampleTime.milliseconds() > waitTime)
            {
                time[index] = runtime.milliseconds(); // record the time at the new index value
                motorPos[index] = motor.getCurrentPosition(); // record the motor position at the new index value
                busVoltage[index] = ina.busVoltage(); // record the bus voltage (from sensor INA219) at the new index value
                //filter.addNewValue(ina.busVoltage());
                shuntCurrent[index] = ina.current(); // record the shunt current (from sensor INA219) at the new index value
                index++; // increment the index value
                sampleTime.reset(); // reset the sample timer
            }
        }
    }
    public void TimedSamplingTest2()
    {
        index = 0; // reset the index value to 0
        sampleTime.reset(); // reset the sample timer
        while (runtime.milliseconds() < 30000 + 60000) // while the motor hasn't been running for 60 seconds...
        {
            // if the sample time is less than the intended time between each sample...
            if (sampleTime.milliseconds() > waitTime)
            {
                time2[index] = runtime.milliseconds(); // record the time at the new index value
                motorPos2[index] = motor.getCurrentPosition(); // record the motor position at the new index value
                busVoltage2[index] = ina.busVoltage(); // record the bus voltage (from sensor INA219) at the new index value
                //filter.addNewValue(ina.busVoltage());
                shuntCurrent2[index] = ina.current(); // record the shunt current (from sensor INA219) at the new index value
                index++; // increment the index value
                sampleTime.reset(); // reset the sample timer
            }
        }
    }


    /*
    This method is for reading and displaying the encoder counts while manually spinning the
    flywheel.  The purpose of this is so that you can compare how much the motor turns optically and
    the encoder count reading.
     */
    public void EncoderCountTest()
    {
        //relay.setState(false); // turn the relay off, even though we know we won't run the motor
        while (opModeIsActive())
        {
            telemetry.addData("EncoderCount", motor.getCurrentPosition()); // read encoder counts to update the count displayed
            telemetry.update(); // display the encoder count to the driver station phone screen
        }
    }

    /*
    This method records the data from the arrays to a a text file using the FileWriter class,
    located in the same folder, TeamSwerve.
     */
    public void RecordData()
    {
        // After the values are recorded into their own arrays, use FileWriter to log the files.
        // Open a file for writing our data into.
        // The file will appear on the robot phone in the folder storage/legacy/emulated
        FileWriter myFile = new FileWriter("testfile.txt");

        // write some data to a file
        for (int i = 0; i < index; i++)
        {
            // pass in time, motor position, bus voltage, and shunt voltage (displayed in that order)
            myFile.println( time[i] + " " + motorPos[i] + " " + busVoltage[i] + " " + shuntCurrent[i] );
        }
        // Close the file when you're done. (Not strictly necessary, but nice to do.)
        myFile.closeFile();
    }

    /*
    This method is the same as the method above, only named with a "2" to indicate that it is used
    to record only data from the arrays from after the relay was turned off.
     */
    public void RecordData2()
    {
        // After the values are recorded into their own arrays, use FileWriter to log the files.
        // Open a file for writing our data into.
        // The file will appear on the robot phone in the folder storage/legacy/emulated
        FileWriter myFile = new FileWriter("testfile2.txt");

        // write some data to a file
        for (int i = 0; i < index; i++)
        {
            myFile.println( time2[i] + " " + motorPos2[i] + " " + busVoltage2[i] + " " + shuntCurrent2[i] );
        }
        // Close the file when you're done. (Not strictly necessary, but nice to do.)
        myFile.closeFile();
    }

    /*
    This test runs the DUT under constant speed.  This speed can be adjusted with a Logitech gamepad
    The motor starts and stops with a toggle button "A".
     */
    public void ConstantVelocityTest()
    {
        while (opModeIsActive())
        {
            // select a motor speed
            if (gamepad1.dpad_up && !isDpadUpPushed && ticksPerSec<2299.999)
            {
                isDpadUpPushed = true;
                ticksPerSec += 100.0;
                motorEx.setVelocity(ticksPerSec);
            }
            isDpadUpPushed = gamepad1.dpad_up;

            if (gamepad1.dpad_down && !isDpadDownPushed && ticksPerSec>0.001)
            {
                isDpadDownPushed = true;
                ticksPerSec -= 100.0;
                motorEx.setVelocity(ticksPerSec);
            }
            isDpadDownPushed = gamepad1.dpad_down;

            // toggle button "A"
            if (gamepad1.a && !isAPressed)
            {
                isAPressed = true;
                isMotorOn = !isMotorOn;
            }
            isAPressed = gamepad1.a;

            if (isMotorOn) // if button "A" is pressed, then set the motor power
            {
                motorEx.setVelocity(ticksPerSec);
                motorEx.setMotorEnable();
                telemetry.addData("Motor", "is on.");
            }
            else
            {
                motorEx.setVelocity(0.0);
                telemetry.addData("Motor", "is off.");
            }

            telemetry.addData("Set Motor ticks per sec: ", ticksPerSec);
            //telemetry.addData("EncoderCount", motorEx.getCurrentPosition());
            telemetry.addData("Velocity ticks per sec: ", motorEx.getVelocity());
            telemetry.update();
        }
    }

    /*
    This method initializes the relay circuit.
     */
    public void initializeRelay()
    {
        // maps general purpose digital pin
        relay = hardwareMap.digitalChannel.get("relay");
        // specify mode is output
        relay.setMode(DigitalChannel.Mode.OUTPUT);
        // true activates the pin, false deactivates the pin
        relay.setState(false); // turn off the relay to initialize
    }

/*
This is where the OpMode starts, including the initializing process.  The runOpMode is not a loop.
 */
    public void runOpMode()
    {
        // Connect to motor
        // Tell the user that the motor is initializing
        telemetry.addData("Motor", "Initializing");
        telemetry.update();

        motorEx = (DcMotorEx)hardwareMap.dcMotor.get("motorEx");
        motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorEx.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motorEx.setDirection(DcMotorEx.Direction.FORWARD);

        // Tell the user that the motor is done initializing
        telemetry.addData(">", "Motor done initializing");
        telemetry.addData("Please select a motor speed using D-Pad (up & down).", "Start/Stop the motor with button 'A'.");
        telemetry.update();

        waitForStart();
        ConstantVelocityTest();
        /*
        motorEx.setVelocity(9.0);
        sleep(5000);
        motorEx.setVelocity(18.0);
        sleep(5000);
        motorEx.setVelocity(1.0);
        sleep(5000);
        motorEx.setVelocity(0.0);
        */

    }
}