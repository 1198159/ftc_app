package org.firstinspires.ftc.team417_2018;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestOpModeTest")
//@Disabled
public class TestOpMode extends LinearOpMode {

    private DcMotor FrontLeftMotor;
    private DcMotor BackRightMotor;
    private DcMotor BackLeftMotor;
    private DcMotor FrontRightMotor;

    @Override


    public void runOpMode() throws InterruptedException
    {
        FrontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");
        FrontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        BackLeftMotor = hardwareMap.dcMotor.get("BackLeft");
        BackRightMotor = hardwareMap.dcMotor.get("BackRight");

        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        FrontLeftMotor.setPower(0.25);
        FrontRightMotor.setPower(0.25);
        BackRightMotor.setPower(0.25);
        BackLeftMotor.setPower(0.25);


        sleep(3000);

        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
    }
}
