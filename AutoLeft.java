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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoLeft", group="Robot")
public class AutoLeft extends LinearOpMode {

    // Declare Motors, Servos, etc.
    private DcMotor leftMotor, rightMotor, slideMotor, middle1, middle2;
    private Servo grabber;
    private double ENCODER_TICKS_PER_ROTATION = 1120;
    private double DISTANCE_PER_ROTATION = 12.57;
    private double driveFactor = 1;
    private ElapsedTime runtime = new ElapsedTime();

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


        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setPower(0.25);
        leftMotor.setTargetPosition(0);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle2.setPower(0.25);
        middle2.setTargetPosition(0);
        middle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middle1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle1.setPower(0.25);
        middle1.setTargetPosition(0);
        middle1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setPower(0.25);
        rightMotor.setTargetPosition(0);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//Repeat for all four motors

        waitForStart();

        runtime.reset();
        grabber.setPosition(.91);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(15));
        driveToPositionHorizontal(1);
        sleep(2000);
        grabber.setPosition(.91);
        sleep(1000);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(200));
        sleep(1000);
        driveToPositionVertical(16);
        sleep(3000);
        driveToPositionHorizontal(60.5);
        sleep(6000);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1325.0));
        sleep(2000);
        driveToPositionVertical(-3);
        sleep(2000);
        grabber.setPosition(0);
        sleep(500);
        driveToPositionVertical(3);
        sleep(2000);
        slideMotor.setTargetPosition(convertDegreesToEncoderTicks(25));
        sleep(3000);
        driveToPositionHorizontal(-14);
        sleep(6000);
        /*driveToPositionVertical(26);
       sleep(3000);
        driveToPositionVertical(-20);
        sleep(3000);
       slideMotor.setTargetPosition(convertDegreesToEncoderTicks(1285.0));
        telemetry.addData(",iddleencoderposition",middle2.getCurrentPosition());
        telemetry.update();
        sleep(10000);
        driveToPositionVertical(-20);
        sleep(3000);
        rotateInDegrees(360);
        sleep(10000);*/



    }

    //Drive Left/Right
    public void driveToPositionHorizontal(double distance) {

        //Convert that distance to encoder ticks
        int encoderTicksRequired2 = (int) ((538/DISTANCE_PER_ROTATION)*distance);
        middle1.setTargetPosition(middle1.getCurrentPosition() + encoderTicksRequired2);
        middle2.setTargetPosition(middle2.getCurrentPosition() + encoderTicksRequired2);



    }

    //Drive Forward/Back
    public void driveToPositionVertical(int distance) {

        int encoderTicksRequired4 = (int) (1120/DISTANCE_PER_ROTATION)*distance;
        leftMotor.setTargetPosition(leftMotor.getCurrentPosition() + encoderTicksRequired4);
        rightMotor.setTargetPosition(rightMotor.getCurrentPosition() + encoderTicksRequired4);


    }


    //Rotate a certain amount of degrees
    public void rotateInDegrees(int degrees) {
       // int fractionofrotation = degrees/360;
        //double convertrotationfraction = fractionofrotation*51.52;
        //int encoderTicksRequired4 = (int) ((1120/DISTANCE_PER_ROTATION)*convertrotationfraction);
        double encoderTicksRequired4 = (1120/360)*degrees;
        leftMotor.setTargetPosition((int) (leftMotor.getCurrentPosition() + encoderTicksRequired4));
        rightMotor.setTargetPosition((int) (rightMotor.getCurrentPosition() - encoderTicksRequired4));

    }
    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees/360 * ENCODER_TICKS_PER_ROTATION);
    }
}
