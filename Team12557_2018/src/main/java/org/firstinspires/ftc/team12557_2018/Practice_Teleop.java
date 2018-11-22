/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Practice_Teleop", group="Pushbot")
//@Disabled
public class Practice_Teleop extends LinearOpMode {

    /* Declare OpMode members. */
    HardwarePractice robot           = new HardwarePractice();   // Use a Pushbot's hardware
                                                               // could also use HardwarePushbotMatrix class.
    @Override
    public void runOpMode() {
        double left;
        double leftY;
        double leftX;
        double rightX;
        double armMotorPos;
        double holdPos;
        boolean isHolding = false;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);


//        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //Andymark 1120 per revolution.


            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.
            leftY = gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;
            
            if (gamepad1.a) {
                isHolding = !isHolding;
                telemetry.update();
            }

            //Movement using joysticks

            if (leftX != 0 || leftY != 0 || rightX != 0) {
                //Forward Backward
                if(leftX < .2 && leftX > -.2) {
                    robot.leftFrontDrive.setPower(-leftY);
                    robot.rightFrontDrive.setPower(-leftY);
                    robot.leftBackDrive.setPower(-leftY);
                    robot.rightBackDrive.setPower(-leftY);
                }
                //Strafing
                if(leftY < .2 && leftY > - .2) {
                    robot.leftFrontDrive.setPower(-leftX);
                    robot.rightFrontDrive.setPower(leftX);
                    robot.leftBackDrive.setPower(leftX);
                    robot.rightBackDrive.setPower(-leftX);
                }
                //Turning
                robot.leftFrontDrive.setPower(rightX);
                robot.rightFrontDrive.setPower(-rightX);
                robot.leftBackDrive.setPower(rightX);
                robot.rightBackDrive.setPower(-rightX);
            } else {
                // Output the safe vales to the motor drives.
                robot.leftFrontDrive.setPower(0);
                robot.rightFrontDrive.setPower(0);
                robot.leftBackDrive.setPower(0);
                robot.rightBackDrive.setPower(0);
            }


            armMotorPos = robot.armDrive.getCurrentPosition();

            if (armMotorPos < 3100) {
                if (gamepad1.right_bumper == true) {
                    robot.armDrive.setPower(1);
                } else if (gamepad1.left_bumper == true) {
                    robot.armDrive.setPower(-1);
                } else if (gamepad1.left_bumper == false || gamepad1.right_bumper == false) {
                    robot.armDrive.setPower(0);
                }
            } else {
                if (gamepad1.left_bumper == true) {
                    robot.armDrive.setPower(-1);
                } else if (gamepad1.left_bumper == false) {
                    robot.armDrive.setPower(0);
                }
            }

            if (armMotorPos > -20) {
                if (gamepad1.right_bumper == true) {
                    robot.armDrive.setPower(1);
                } else if (gamepad1.left_bumper == true) {
                    robot.armDrive.setPower(-1);
                } else if (gamepad1.left_bumper == false || gamepad1.right_bumper == false) {
                    robot.armDrive.setPower(0);
                }
            } else {
                if (gamepad1.right_bumper == true) {
                    robot.armDrive.setPower(1);
                } else if (gamepad1.right_bumper == false) {
                    robot.armDrive.setPower(0);
                }
            }
            
            if (isHolding) {
                holdPos = armMotorPos;
                robot.armDrive.setTargetPosition((int) holdPos);
                robot.armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.armDrive.setPower(.5);
            } else {
                robot.armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Send telemetry message to signify robot running;
            telemetry.addData("left stick gamepad 1 X: ",  "%.2f", leftX );
            telemetry.addData("left stick gamepad 1 Y: ", "%.2f", leftY);
            telemetry.addData("right stick gamepad 1 X: ", "%.2f", rightX);
            telemetry.addData("Arm Motor position: ", "%.2f", armMotorPos);
            telemetry.addData("Is Holding: ", isHolding);
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.
            //sleep(50);
        }
    }
}
