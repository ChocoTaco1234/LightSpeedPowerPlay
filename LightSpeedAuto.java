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

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive LeftBack", group="Robot")

public class LightSpeedAuto extends LinearOpMode {

    //Name all your motors and servos (basically tell the code that these exist)
    private DcMotor leftMotor, rightMotor, armMotor, middle1, middle2;
    private Servo grabber, rotator;
    private double ENCODER_TICKS_PER_ROTATION = 1120;
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        //telling the robot what is what for motors and servos
        leftMotor = hardwareMap.dcMotor.get("left motor");
        rightMotor = hardwareMap.dcMotor.get("right motor"); //Gets encoder
        armMotor = hardwareMap.dcMotor.get("arm motor");
        grabber = hardwareMap.servo.get("grabber");
        rotator = hardwareMap.servo.get("grabber rotator");
        middle1 = hardwareMap.dcMotor.get("middle1");
        middle2 = hardwareMap.dcMotor.get("middle2"); //Gets encoder

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Stop and set the position you are at to 0, then whenever I tell you to go to a location, do it at 0.25 power
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(0.25);

        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setTargetPosition(convertDegreesToEncoderTicks(0));
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(0.5);

        middle2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle2.setTargetPosition(convertDegreesToEncoderTicks(0));
        middle2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middle2.setPower(0.5);



        //Closes the grabber
        grabber.setPosition(0.31);

        /***
         * THIS IS WHERE IT ACTUALLY ALL STARTS. EVERYTHING AFTER THE waitForStart() IS WHAT HAPPENS WHEN YOU RUN AUTO. EVERYTHING
         * BEFORE IS IN INIT
         ***/
        //Wait to execute anything below this line until autonomous starts
        waitForStart();

        runtime.reset();
        grabber.setPosition(0.31); //Make sure grabber is closed

        //Move arm up and set rotator to match arm
        armMotor.setTargetPosition(convertDegreesToEncoderTicks(80.0)); //Move the arm up
        sleep(500);
        rotator.setPosition(.66);

        sleep(1000);

        grabber.setPosition(0.05); //Open the grabber and deposit the cone

        sleep(1000);

        armMotor.setTargetPosition(convertDegreesToEncoderTicks(0)); //Bring the arm down by setting it to position 0
        rotator.setPosition(.30); //Set the rotator to rest position

        sleep(1000);
        //Grabber let go

        rotator.setPosition(0);
        sleep(1000);
        rightMotor.setTargetPosition(convertDegreesToEncoderTicks(-200));
        sleep(1000);
        middle2.setTargetPosition(820); //Middle wheel gets 537.6 encoder ticks per rotation 1250/537.6 = 2.1 rotations

        middle1.setPower(middle2.getPower()); //Set the other middle wheel to the same power as middle2
        sleep(1000);
        rightMotor.setPower(0);
        middle1.setPower(0);
        middle2.setPower(0);

    }

    public int convertDegreesToEncoderTicks(double degrees) {
        return (int) (degrees/360 * ENCODER_TICKS_PER_ROTATION);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */

}
