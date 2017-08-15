package org.firstinspires.ftc.teamswerve; // This software belongs in the Team Swerve Folder
// Below are all of the imports from other classes including DcMotor, DigitalChannel, and ElapsedTime.
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

// The name of this test is "DynamomterTests", and it belongs to the group "Swerve".
@TeleOp(name="DynamomterTests", group = "Swerve")
// @Disabled // when NOT disabled, "DynamomterTests" will show up on the select OpMode screen, under "TeleOp"
public class DynamomterProject extends LinearOpMode // "DynamometerProject" is a subclass of the base class "LinearOpMode".
{

    DcMotor motor; // We are testing AndyMark NeverRest 20s, 40s, 60s, 3.7s, Matrix, and REV Core Hex motors (three of each).
    DigitalChannel relay; // Relay circuit, plugged into port 0 of the REV module
    INA219 ina; // INA219 Current Sensor, plugged into port 1 of the REV module.  Here's the link to the data sheet: https://cdn-shop.adafruit.com/datasheets/ina219.pdf

    double motorPower = 0.0; // used as a placeholder value for incremented motor power the method "RampUpMotor" below
    int waitTime = 100; // a value used for time between each motor power increment in the method "RampUpMotor" below
    double motorSetPower = 0.4; // the max power the motor power will increment to in the method "RampUpMotor" below
    int numSamples = 1500; // the maximum number of data samples the arrays for time, encoder counts, current, and voltage will store
    // values before the relay turns off
    int currentPos; // the current encoder position
    double shuntVoltageValue; // the value of the shunt voltage read from the INA219 current sensor
    double busVoltageValue; // the value of the bus voltage read from the INA219
    // values after the relay turns off, named the same as the three variables above,
    // just with a "2" at the end to indicate they're used after the relay is turned off
    int currentPos2;
    double shuntVoltageValue2;
    double busVoltageValue2;

    private ElapsedTime runtime = new ElapsedTime(); // used for timer that starts as soon as the play button is pushed
    int index = 0; // used to keep track of arrays
    // declare parallel arrays, made parallel by the same index
    double[] time = new double[numSamples]; // logs time every time a new encoder value is reached
    int[] motorPos = new int[numSamples]; // logs encoder counts
    double[] shuntVoltage = new double[numSamples]; // logs current every time a new encoder value is reached
    double[] busVoltage = new double[numSamples]; // logs voltage every time a new encoder value is reached
    // second set for logging after the relay is turned off, named the same as the four arrays above,
    // just with a "2" at the end to indicate they're used after the relay is turned off
    double[] time2 = new double[numSamples];
    int[] motorPos2 = new int[numSamples];
    double[] shuntVoltage2 = new double[numSamples];
    double[] busVoltage2 = new double[numSamples];

    /*
    This method counts the number of times the encoder count changes, and records the time with
    that certain index into an array.  There are two arrays that are parallel to each other by the
    same index: "time", which holds the time that the motor encoder count changed, and "motorPos",
    which holds the encoder reading.
    */
    public void MotorTest()
    {
        /*
        Set the first value of motorPos and time to their respective readings when the test
        begins, before the runtime starts.
        */
        motorPos[0] = motor.getCurrentPosition(); // retrieve the motor's current encoder value
        time[0] = runtime.milliseconds(); // set the first value of time array to the current time using runtime object
        busVoltage[0] = ina.busVoltage(); // record the first value of the current array to the current current reading from the INA219
        shuntVoltage[0] = ina.shuntVoltage(); // record the first value the voltage array to the current shunt voltage from the INA219

        index = 0; // reset the index value to 0
        while (runtime.milliseconds() < 20000) // while the test has been under 20 seconds...
        {
            /*
            Check the current position and if it's not the same as the last motor position, then
            increment the index, record the time to the array and record the motor position.  This
            whole process will keep looping until the 20 seconds is up.
            */
            currentPos = motor.getCurrentPosition(); // get the current encoder value
            busVoltageValue = ina.busVoltage(); // get the current value from the INA219 current sensor
            shuntVoltageValue = ina.shuntVoltage(); // get the shunt voltage value from the INA219 current sensor

            // If the current position is not equal to the motor position at the current index value
            // and the current index value is less than the number samples, then...
            if ((currentPos != motorPos[index]) && (index < numSamples))
            {
                index++; // increment the current index value
                time[index] = runtime.milliseconds(); // record the time at the new index value
                motorPos[index] = currentPos; // record the motor position at the new index value
                busVoltage[index] = busVoltageValue; // record the bus voltage (from sensor INA219) at the new index value
                shuntVoltage[index] = shuntVoltageValue; // record the shunt voltage at the new index value
            }
        }
    }

    /*
    The purpose of this test is to calculate the encoder counts vs. time AFTER the relay has been turned off.
    This method is the same as the method above, only with a "2" after the method name to indicate that it is
    to be used after the relay is turned off.  The variables and arrays inside this method follow the same naming
    protocol.
     */
    public void MotorTest2()
    {
        /*
        Set the first value of motorPos and time to their respective readings when the test
        begins, before the runtime starts.
        */
        motorPos2[0] = motor.getCurrentPosition();
        time2[0] = runtime.milliseconds();
        busVoltage2[0] = ina.busVoltage();
        shuntVoltage2[0] = ina.shuntVoltage();

        index = 0;
        while (runtime.milliseconds() < 40000 + 20000) // 40 seconds after the relay is turned off
        {
            currentPos2 = motor.getCurrentPosition();
            busVoltageValue2 = ina.busVoltage();
            shuntVoltageValue2 = ina.shuntVoltage();
            if ((currentPos2 != motorPos2[index]) && (index < numSamples))
            {
                index++;
                time2[index] = runtime.milliseconds();
                motorPos2[index] = currentPos2;
                busVoltage2[index] = busVoltageValue2;
                shuntVoltage2[index] = shuntVoltageValue2;
            }
        }
    }

    /*
    This method ramps up motor power by 1% every 100 milliseconds.  The purpose of this test was to
    safely and gradually run the motor to a set speed.
     */
    public void RampUpMotor()
    {
        while (runtime.milliseconds() < 10000)
        {
            while (motorPower < motorSetPower)
            {
                motorPower += 0.01;
                motor.setPower(motorPower);
                sleep(waitTime);
                telemetry.addData("Power:", motor.getPower());
                telemetry.update();
            }
        }
        motor.setPower(0.0);
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
            myFile.println( time[i] + " " + motorPos[i] + " " + busVoltage[i] + " " + shuntVoltage[i] );
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
            myFile.println( time2[i] + " " + motorPos2[i] + " " + busVoltage2[i] + " " + shuntVoltage2[i] );
        }
        // Close the file when you're done. (Not strictly necessary, but nice to do.)
        myFile.closeFile();
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
        relay.setState(false);
    }

/*
This is where the OpMode starts, including the initializing process.
 */
    public void runOpMode()
    {
        // Connect to motor
        motor = hardwareMap.dcMotor.get("motor");
        initializeRelay();

        //motor.setDirection(DcMotor.Direction.REVERSE); // reverse the motor
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // allows the motor to slow down without brakes
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // this mode simply inputs power, so no PID

        // create current sensor object
        ina = hardwareMap.get(INA219.class, "ina");
        ina.doInitialize(); // initialize the INA219 current sensor

        telemetry.addData(">", "Press start to run Motor"); // write a message to indicate the initialization is done
        telemetry.update(); // update the message to display on the UI (the very bottom of the driver station phone screen)

        waitForStart(); // wait until the start button is pushed

        runtime.reset(); // restart the timer (used when running the motor for seconds
        relay.setState(true); // turn relay on

        //RampUpMotor(); // Ramp up the motor speed for 10 seconds

        //motor.setPower(motorSetPower); // turn on the motor to the set power
        MotorTest(); // record encoder counts vs. time as well as current and voltage
        //motor.setPower(0.0); // turn the motor off

        relay.setState(false); // turn relay off after 20 seconds

        // Record data into arrays after the relay is turned off
        MotorTest2();

        // Record the Data from the arrays
        RecordData();
        RecordData2();

        telemetry.addData( ">", "Press Stop to end test." ); // write final message to indicate the test is over
        telemetry.update(); // update the message to the driver station phone screen

        // for safety, just in case the relay doesn't power off by itself
        relay.setState(false); // turn relay off
    }
}