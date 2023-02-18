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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Two Driver OpMode", group="Linear Opmode")
public class LightSpeedTeleOp2Drivers extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private DcMotor leftMotor, rightMotor, slideMotor, middle1, middle2;
    private Servo grabber;
    private double ENCODER_TICKS_PER_ROTATION = 1120;
    private double driveFactor = 1;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor"); //Gets encoder
        slideMotor = hardwareMap.dcMotor.get("slide motor");
        grabber = hardwareMap.servo.get("grabber");
        middle1 = hardwareMap.dcMotor.get("middle1");
        middle2 = hardwareMap.dcMotor.get("middle2");

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.75);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.a) {
                slideMotor.setPower(0.4);
                slideMotor.setTargetPosition(convertDegreesToEncoderTicks(45.0));

            } else if (gamepad2.x) {
                slideMotor.setPower(0.8);
                slideMotor.setTargetPosition(convertDegreesToEncoderTicks(641.0));
            }
            else if (gamepad2.y) {
                slideMotor.setPower(0.8);
                slideMotor.setTargetPosition(convertDegreesToEncoderTicks(986.0));
            }
            else if (gamepad2.b) {
                slideMotor.setPower(0.8);
                slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1307.0));
            }
            else if (gamepad2.dpad_down) {
                slideMotor.setPower(.4);
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() - (convertDegreesToEncoderTicks(50)));
            }
            else if (gamepad2.dpad_up) {
                slideMotor.setPower(.4);
                slideMotor.setTargetPosition(slideMotor.getCurrentPosition() + (convertDegreesToEncoderTicks(50)));

            }




            if (gamepad2.right_trigger > .5) {
                grabber.setPosition(.91);
            } else {
                grabber.setPosition(0);
            }

            if (gamepad1.left_bumper) {
                driveFactor = 1;
            }
            if (gamepad1.right_bumper) {
                driveFactor = 0.5;
            }

            leftMotor.setPower(driveFactor * gamepad1.left_stick_y);
            rightMotor.setPower(driveFactor * gamepad1.right_stick_y);


                middle1.setPower(driveFactor * (gamepad1.right_trigger-gamepad1.left_trigger)/2);
                middle2.setPower(driveFactor * (gamepad1.right_trigger-gamepad1.left_trigger)/2);


            }

        }



    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees/360 * ENCODER_TICKS_PER_ROTATION);
    }
}
