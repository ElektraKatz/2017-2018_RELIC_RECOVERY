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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 * <p>
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "TestDriveMecanumComp", group = "Test")
//@Disabled
public class TestDriveMecanum_Comp extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareCompBot robot = new HardwareCompBot();              // Use a K9'shardware

    @Override
    public void runOpMode() {
        double dblLeftX = 0;
        double dblLeftY = 0;
        double dblRightX = 0;
        double dblRawLeftMotor1;
        double dblRawLeftMotor2;
        double dblRawRightMotor1;
        double dblRawRightMotor2;
        double dblLeftMotor1Power;
        double dblLeftMotor2Power;
        double dblRightMotor1Power;
        double dblRightMotor2Power;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            dblLeftY = gamepad1.left_stick_y;
            dblLeftX = gamepad1.left_stick_x;
            dblRightX = gamepad1.right_stick_x;

            dblRawLeftMotor1 = -dblLeftY + dblLeftX + dblRightX;
            dblRawLeftMotor2 = -dblLeftY - dblLeftX + dblRightX;
            dblRawRightMotor1 = -dblLeftY - dblLeftX - dblRightX;
            dblRawRightMotor2 = -dblLeftY + dblLeftX - dblRightX;

            dblLeftMotor1Power = Range.clip(dblRawLeftMotor1, -1.0, 1.0);
            dblLeftMotor2Power = Range.clip(dblRawLeftMotor2, -1.0, 1.0);
            dblRightMotor1Power = Range.clip(dblRawRightMotor1, -1.0, 1.0);
            dblRightMotor2Power = Range.clip(dblRawRightMotor2, -1.0, 1.0);

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", " LY=" + dblLeftY + " LX=" + dblLeftX + " RX=" + dblRightX);
            telemetry.addData("raw", " L1=" + dblRawLeftMotor1 + " L2=" + dblRawLeftMotor2 + " R1=" + dblRawRightMotor1 + " R2=" + dblRawRightMotor2);
            telemetry.addData("power", " L1=" + dblLeftMotor1Power + " L2=" + dblLeftMotor2Power + " R1=" + dblRightMotor1Power + " R2=" + dblRawRightMotor2);
            telemetry.update();

            robot.leftMotor1.setPower(dblLeftMotor1Power);
            robot.leftMotor2.setPower(dblLeftMotor2Power);
            robot.rightMotor1.setPower(dblRightMotor1Power);
            robot.rightMotor2.setPower(dblRightMotor2Power);



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}
