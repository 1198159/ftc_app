package org.firstinspires.ftc.team417_2018;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BC Autonomous")
//@Disabled
public class TestOpMode extends MasterAutonomous {

    public void runOpMode() throws InterruptedException
    {
        super.initializeHardware();

        telemetry.addData("Init:", "done");
        telemetry.update();
        waitForStart();

        double refAngle = imu.getAngularOrientation().firstAngle;
        land();
        move(-150, 0, 0.2, 0.7, 1.0);
        //unhook();
      //  pivotWithReference(90, refAngle, 0.2, 0.8);

        /*
        move(200, 0, 0.3, 0.7, 1.0);
        sleep(200);
        move(0, 200, 0.3, 0.7, 1.0);
        sleep(200);
        move(-200, 0, 0.3, 0.7, 1.0);
        sleep(200);
        */
        //move(0, -1000, 0.3, 0.7, 3.0);
    }
}
