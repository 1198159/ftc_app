package org.firstinspires.ftc.team8923;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Encoder Test", group = "Tests")
@Disabled
public class TestEncoder extends MasterTeleOp
{
    ElapsedTime time = new ElapsedTime();
    int ticksFL = 0;
    int ticksFR = 0;
    int ticksBL = 0;
    int ticksBR = 0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initHardware();

        waitForStart();

        time.reset();

        while(opModeIsActive())
        {
            if(gamepad1.a)
            {
                motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if(gamepad1.b)
            {
                motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(gamepad1.x)
            {
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if(gamepad1.y)
            {
                motorFL.setPower(1.0);
                motorFR.setPower(1.0);
                motorBL.setPower(1.0);
                motorBR.setPower(1.0);
                time.reset();

                while(time.milliseconds() < 500)
                {
                    idle();
                }
                int lastFL = motorFL.getCurrentPosition();
                int lastFR = motorFR.getCurrentPosition();
                int lastBL = motorBL.getCurrentPosition();
                int lastBR = motorBR.getCurrentPosition();
                time.reset();
                while(time.milliseconds() < 1000)
                {
                    idle();
                }
                ticksFL = motorFL.getCurrentPosition() - lastFL;
                ticksFR = motorFR.getCurrentPosition() - lastFR;
                ticksBL = motorBL.getCurrentPosition() - lastBL;
                ticksBR = motorBR.getCurrentPosition() - lastBR;
            }

            driveMecanumTeleOp();

            telemetry.addData("PowerFL", motorFL.getPower());
            telemetry.addData("PowerFR", motorFR.getPower());
            telemetry.addData("PowerBL", motorBL.getPower());
            telemetry.addData("PowerBR", motorBR.getPower());

            telemetry.addData("EncoderFL", motorFL.getCurrentPosition());
            telemetry.addData("EncoderFR", motorFR.getCurrentPosition());
            telemetry.addData("EncoderBL", motorBL.getCurrentPosition());
            telemetry.addData("EncoderBR", motorBR.getCurrentPosition());

            telemetry.addData("TicksFL", ticksFL);
            telemetry.addData("TicksFR", ticksFR);
            telemetry.addData("TicksBL", ticksBL);
            telemetry.addData("TicksBR", ticksBR);

            telemetry.addData("StickX", gamepad1.left_stick_x);
            telemetry.addData("StickY", gamepad1.left_stick_y);

            telemetry.update();
            idle();
        }
    }
}
