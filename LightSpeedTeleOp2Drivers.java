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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Two Driver OpMode", group="Linear Opmode")
public class LightSpeedTeleOp2Drivers extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private DcMotor leftMotor, rightMotor, armMotor, middle1, middle2;
    private Servo grabber, rotator;
    private double ENCODER_TICKS_PER_ROTATION = 1120;
    private double driveFactor = 1;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor");
        armMotor = hardwareMap.dcMotor.get("arm motor");
        grabber = hardwareMap.servo.get("grabber");
        rotator = hardwareMap.servo.get("grabber rotator");
        middle1 = hardwareMap.dcMotor.get("middle1");
        middle2 = hardwareMap.dcMotor.get("middle2");

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.25);

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        grabber.setPosition(.20);
        rotator.setPosition(0);

        waitForStart();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad2.a) {
                armMotor.setTargetPosition(convertDegreesToEncoderTicks(0.0));
                rotator.setPosition(.35);
            } else if (gamepad2.x) {
                armMotor.setTargetPosition(convertDegreesToEncoderTicks(90.0));
                sleep(500);
                rotator.setPosition(.60);
            } else if (gamepad2.y) {
                armMotor.setTargetPosition(convertDegreesToEncoderTicks(135.0));
                sleep(500);
                rotator.setPosition(.80);
            } else if (gamepad2.b) {
                armMotor.setTargetPosition(armMotor.getCurrentPosition() - (convertDegreesToEncoderTicks(5)));
            }

            if (gamepad2.right_trigger > .5) {
                grabber.setPosition(.31);
            } else {
                grabber.setPosition(0.05);
            }

            if (gamepad1.left_bumper) {
                driveFactor = 1;
            }
            if (gamepad1.right_bumper) {
                driveFactor = 0.5;
            }

            leftMotor.setPower(driveFactor * gamepad1.left_stick_y);
            rightMotor.setPower(driveFactor * gamepad1.right_stick_y);

            if (gamepad1.right_trigger > 0) {
                middle1.setPower(driveFactor * gamepad1.right_trigger/2);
                middle2.setPower(driveFactor * gamepad1.right_trigger/2);
            } else {
                middle1.setPower(driveFactor * -gamepad1.left_trigger/2);
                middle2.setPower(driveFactor * -gamepad1.left_trigger/2);
            }

        }
    }

    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees/360 * ENCODER_TICKS_PER_ROTATION);
    }
}
