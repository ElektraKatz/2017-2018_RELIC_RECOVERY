/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
//                                                   DO NOT CHANGE

/**
 * This file contains the TeleOp code for FTC 11230 ElektraKatz for Velocity Vortex competition
 * The robot uses the TileRunner BD base with team built accessories attached to it.
 * The drive system consists of four AndyMark NeveRest 60 motors (2 left, 2 right)
 * The lift is driven by 1 AndyMark NeveRest 40 motor
 * <p>
 * The motors are defined as follows:
 * leftMotor1   = "left motor1"
 * leftMotor2   = "left motor2"
 * rightMotor1  = "right motor1"
 * rightMotor2  = "right motor2"
 * upMotor      = "up motor"
 * <p>
 * This OpMode started with the FIRST example "TemplateOpMode_Linear"
 * Team member Conlan Houston modified the Template code to create this TeleOp OpMode
 */
@Disabled
@TeleOp(name = "Conmanbot: Linear OpMode", group = "ElektraKatz")
public class chouston2 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor1 = null;
    DcMotor rightMotor1 = null;
    DcMotor leftMotor2 = null;
    DcMotor rightMotor2 = null;
//    DcMotor upMotor = null;
//    public Servo leftClaw = null;
//    public Servo beaconHit = null;
//    public Servo beaconHit2 = null;
//    public Servo ballShoot = null;

    //    public Servo    frontClaw   = null;
    static double CLAW_OPEN = 0;
    static double CLAW_CLOSE = 1;
    static double CLAW_MID = 0.5;

    boolean oneShot = false;
    boolean oneShotb = false;
    boolean oneShotb2 = false;
    //    boolean         oneShotc     = false;
    boolean oneShotd = false;
    boolean driveForward = true;
//    boolean         oneShotd2     = false;
//    boolean         oneShotd3     = false;
//    boolean         oneShotd4     = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotor1 = hardwareMap.dcMotor.get("left motor1");
        leftMotor2 = hardwareMap.dcMotor.get("left motor2");
        rightMotor1 = hardwareMap.dcMotor.get("right motor1");
        rightMotor2 = hardwareMap.dcMotor.get("right motor2");
//        upMotor = hardwareMap.dcMotor.get("up motor");
//        leftClaw = hardwareMap.servo.get("actuator");
//        beaconHit = hardwareMap.servo.get("actuator2");
//        beaconHit2 = hardwareMap.servo.get("actuator3");
//        ballShoot = hardwareMap.servo.get("actuator2");
//        frontClaw   = hardwareMap.servo.get("actuator3");

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        leftMotor1.setDirection(DcMotor.Direction.REVERSE);
        leftMotor2.setDirection(DcMotor.Direction.REVERSE);// Set to REVERSE if using AndyMark motors
//        upMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
//        leftClaw.setPosition(CLAW_CLOSE);//Initiate left claw to 0 position
//        ballShoot.setPosition(CLAW_CLOSE);
        //frontClaw.setPosition(CLAW_OPEN);

//        upMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
//            upMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            rightMotor1.setPower(-gamepad1.right_stick_y);
            rightMotor2.setPower(-gamepad1.right_stick_y);
            leftMotor1.setPower(-gamepad1.left_stick_y);
            leftMotor2.setPower(-gamepad1.left_stick_y);
////            if (gamepad2.left_stick_y < 0 && upMotor.getCurrentPosition() < 10000) {
//                upMotor.setPower(gamepad2.left_stick_y);
//            } else if (gamepad2.left_stick_y>=0) {
//                upMotor.setPower(gamepad2.left_stick_y);
//            }

//            upMotor.setPower(gamepad2.left_stick_y);
//            telemetry.addData("Lift", "Position at %7d :%7d",
//                    upMotor.getCurrentPosition());
//                    upMotor.getCurrentPosition();
//            telemetry.update();

// 1~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//            if (upMotor.getCurrentPosition() < 10000) {
//                oneShotd = false;
//            }
//            if (upMotor.getCurrentPosition() < 10000) {
//                oneShotd = false;
//            }
//            sleep(20000);
//            upMotor.setPower(gamepad1.left_stick_y);
//            telemetry.update();
////
//            if (gamepad1.x && !oneShotd) {
//
//                oneShotd = true;
//                telemetry.addData("Status:", "'gamepad2.x' pressed");
//
//                if (upMotor.getCurrentPosition()<) {
//                    telemetry.addData("Status:", "Close Gripper");
//                    upMotor.setPower(0);
//                } else if ((upMotor.getCurrentPosition()>)) {
//                    telemetry.addData("Status:", "Open Gripper");
//                    rightMotor1.setPower(-gamepad1.left_stick_y);
//                }
//
////                wait(10);
//            }
//            telemetry.update();

//

//3~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//            if (!gamepad2.a) {
//                oneShot = false;
//            }
//
//
//            if (gamepad2.a && !oneShot) {
//
//                oneShot = true;
//                telemetry.addData("Status:", "'gamepad2.a' pressed");
//
//                if (leftClaw.getPosition() == CLAW_MID) {
//                    telemetry.addData("Status:", "Close Gripper");
//                    leftClaw.setPosition(CLAW_CLOSE);
//                } else if ((leftClaw.getPosition() == CLAW_CLOSE)) {
//                    telemetry.addData("Status:", "Open Gripper");
//                    leftClaw.setPosition(CLAW_MID);
//                }
//
//            }

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//            if (!gamepad2.b) {
//                oneShotb = false;
//            }
//
//
//            if (gamepad2.b && !oneShotb) {
//
//                oneShotb = true;
//                telemetry.addData("Status:", "'gamepad2.b' pressed");
//
//                if (ballShoot.getPosition() == CLAW_MID) {
//                    telemetry.addData("Status:", "Beacon Hit Raise");
//                    ballShoot.setPosition(CLAW_CLOSE);
//                    beaconHit2.setPosition(CLAW_OPEN);
//                } else if ((ballShoot.getPosition() == CLAW_CLOSE)) {
//                    telemetry.addData("Status:", "Hit Beacon");
//                    ballShoot.setPosition(CLAW_MID);
//                    beaconHit2.setPosition(CLAW_CLOSE);
//                }
//                telemetry.update();

//                wait(10);
        }
//            telemetry.update();


//            if (!gamepad2.b) {
//                oneShotb2 = false;
//            }
//
//
//            if (gamepad2.b && !oneShotb2) {
//
//                oneShotb2 = true;
//                telemetry.addData("Status:", "'gamepad2.b' pressed");
//
//                if (beaconHit2.getPosition() == CLAW_CLOSE) {
//                    telemetry.addData("Status:", "Beacon Hit 2 = Raise");
//                    beaconHit2.setPosition(CLAW_OPEN);
//                } else if ((beaconHit2.getPosition() == CLAW_OPEN)) {
//                    telemetry.addData("Status:", "Hit Beacon");
//                    beaconHit2.setPosition(CLAW_CLOSE);
//                }
//                telemetry.update();
//
//            }
//CONTROL~SWITCH~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//
//            if (!gamepad1.x) {
//                oneShotd = false;
//            }
//            if (gamepad1.x && !oneShotd) {
//                 if ((driveForward=false)){
//                    driveForward=true;
//                    telemetry.addData("Direction:", "Forward");
//
//                 }else {
//                    driveForward=false;
//                    telemetry.addData("Direction:", "Reverse");
//                }
//                oneShotd=true;
//                telemetry.update();
//
//            }
//            if (driveForward=true) {
//                rightMotor1.setPower(-gamepad1.left_stick_y);
//                rightMotor2.setPower(-gamepad1.left_stick_y);
//                leftMotor1.setPower(-gamepad1.right_stick_y);
//                leftMotor2.setPower(-gamepad1.right_stick_y);
//
//            } else {
//                rightMotor1.setPower(gamepad1.right_stick_y);
//                rightMotor2.setPower(gamepad1.right_stick_y);
//                leftMotor1.setPower(gamepad1.left_stick_y);
//                leftMotor2.setPower(gamepad1.left_stick_y);
//                }

//            if (!gamepad1.y) {
//                oneShotc = false;
//            }
//

//            if (gamepad1.y && !oneShotc) {
//
//                oneShotc = true;
//                telemetry.addData("Status:", "'gamepad2.y' pressed");
//
//                if (rightClaw.getPosition() == CLAW_OPEN) {
//                    telemetry.addData("Status:", "Close Gripper");
//                    rightClaw.setPosition(CLAW_CLOSE);
//                } else if ((rightClaw.getPosition() == CLAW_CLOSE)) {
//                    telemetry.addData("Status:", "Open Gripper");
//                    rightClaw.setPosition(CLAW_OPEN);
//                }
//
////                wait(10);
//            }
//            telemetry.update();
//
//
//                idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
//            }

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


        idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
    }

}
