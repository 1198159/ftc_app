package org.firstinspires.ftc.teamswerve;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;

@TeleOp(name="DynamomterTests", group = "Swerve")
// @Disabled
public class DynamomterProject extends LinearOpMode
{

    DcMotor motor;
    DigitalChannel relay;
    INA219 ina;
    double motorPower = 0;
    int waitTime = 100;
    double motorSetPower = 0.4;
    int numSamples = 1000000;
    int currentPos;


    private ElapsedTime runtime = new ElapsedTime();
    int index = 0;
    // declare two parallel arrays
    double[] time = new double[numSamples];
    int[] motorPos = new int[numSamples];

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
        motorPos[0] = motor.getCurrentPosition();
        time[0] = runtime.milliseconds();

        index = 0;
        while (runtime.milliseconds() < 5000)
        {
            /*
            Check the current position and if it's not the same as the last motor position, then
            increment the index, record the time to the array and record the motor position.  This
            whole process will keep looping until the 5 seconds is up.
            */
            currentPos = motor.getCurrentPosition();
            if (currentPos != motorPos[index])
            {
                index++;
                time[index] = runtime.milliseconds();
                motorPos[index] = currentPos;
            }
        }
    }


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


    public void RecordData()
    {
        // After the values are recorded into their own arrays, use FileWriter to log the files.
        // Open a file for writing our data into.
        // The file will appear on the robot phone in the folder storage/legacy/emulated
        FileWriter myFile = new FileWriter("testfile.txt");

        //write some data to a file
        for (int i = 0; i < index; i++)
        {
            myFile.println( time[i] + " " + motorPos[i] ); // pass in time and motor position
        }
        // Close the file when you're done. (Not strictly necessary, but nice to do.)
        myFile.closeFile();
    }

    public void initializeRelay()
    {
        // maps general purpose digital pin
        relay = hardwareMap.digitalChannel.get("relay");
        // specify mode is output
        relay.setMode(DigitalChannel.Mode.OUTPUT);
        // true activates the pin, false deactivates the pin
        relay.setState(false);
    }


    public void runOpMode()
    {
        // Connect to motor
        motor = hardwareMap.dcMotor.get("motor");
        initializeRelay();

        motor.setDirection(DcMotor.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // input power, so no PID

        // Wait for the start button
        telemetry.addData(">", "Press start to run Motor");
        telemetry.update();

        waitForStart();

        runtime.reset();
        relay.setState(true); // turn relay on

        //RampUpMotor();

        //motor.setPower(motorSetPower); // turn on the motor to the set power
        //MotorTest();
        //motor.setPower(0.0); // turn the motor off

        //RecordData();

        sleep(5000);

        relay.setState(false); // turn relay off
        telemetry.addData( ">", "Press Stop to end test." );
        telemetry.update();


        while (opModeIsActive())
        {

        }
    }
}