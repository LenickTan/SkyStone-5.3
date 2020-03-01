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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SkystoneTeleop", group="Pushbot")
public class SkystoneTeleop extends OpMode{

    double intakePower = 0;
    double outtakePower = 0;
    double mTapePower = 0;
    boolean front = false;
    boolean slow = false;

    public DcMotor leftFront;       // wheels
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;

    public DcMotor intake1;      //left wheel intake(green wheels)
    public DcMotor intake2;      //right wheel intake(green wheels)

    public DcMotor mTape;       //measuring tape extends over parking zone for point

    //public Servo arm;

    public Servo clasp1;       //grabbing platform; right side?
    public Servo clasp2;       //grabbing platform; left side?



    @Override
    public void init() {
        // initializing motors
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftBack = hardwareMap.dcMotor.get("leftBack");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);  //reverse one side of the robot because of
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);   //motor direction and mecanum wheels

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        intake2.setDirection(DcMotorSimple.Direction.REVERSE);

        mTape = hardwareMap.dcMotor.get("mTape");

        //arm = hardwareMap.servo.get("arm");

        clasp1 = hardwareMap.servo.get("clasp1");
        clasp2 = hardwareMap.servo.get("clasp2");


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        float x;
        float y;
        float z;



        //Gamepad 1 Controls

            //drive train
            if (Math.abs(gamepad1.right_stick_x) > .2) {      // drive train
                x = gamepad1.right_stick_x;
            } else {
                x = 0;
            }

            if (Math.abs(gamepad1.left_stick_y) > .2) {
                y = gamepad1.left_stick_y;
            } else {
                y = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) > .2) {
                z = gamepad1.left_stick_x;
            } else {
                z = 0;
            }


            /* intake wheels run
            left trigger = suck in
            right trigger = spit out */
            if (Math.abs(gamepad1.left_trigger) > .1) {
                intakePower = -1;

            } else if (Math.abs(gamepad1.right_trigger) > .1) {
                intakePower = 1;

            } else {
                intakePower = 0;

            }



        // Gamepad 2 controls


           // a closes foundation movers ; b opens
            if (gamepad2.a) {
                clasp1.setPosition(1);
                clasp2.setPosition(0);
            }

            if (gamepad2.b) {
                clasp1.setPosition(0);
                clasp2.setPosition(1);
            }


            //pulls measuring tape back in
            if (Math.abs(gamepad2.left_trigger) > .1) {
                mTapePower = 1;

            } else if (Math.abs(gamepad2.right_trigger) > .1) {
                mTapePower = -1;

            } else {
                mTapePower = 0;

            }


            if(gamepad1.dpad_left)
                slow = true;
            else if(gamepad1.dpad_right)
                slow = false;

            if(!front && !slow){
                leftBack.setPower(y - x + z);                   // initializing powers
                leftFront.setPower(y - x - z);                  //changes based on dpad control
                rightBack.setPower(y + x - z);                  //left = slower speed
                rightFront.setPower(y + x + z);                 //right = regular speed
            }
            else if(!front && slow){
                leftBack.setPower(0.4*(y - x + z));                   // initializing powers
                leftFront.setPower(0.4*(y - x - z));
                rightBack.setPower(0.4*(y + x - z));
                rightFront.setPower(0.4*(y + x + z));
            }


        intake1.setPower(0.75*intakePower);
        intake2.setPower(0.75*intakePower);

        mTape.setPower(mTapePower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop () {
    }


}
