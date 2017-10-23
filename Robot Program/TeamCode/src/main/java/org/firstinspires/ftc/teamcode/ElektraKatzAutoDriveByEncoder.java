///*
//Copyright (c) 2016 Robert Atkinson
//
//All rights reserved.
//
//Redistribution and use in source and binary forms, with or without modification,
//are permitted (subject to the limitations in the disclaimer below) provided that
//the following conditions are met:
//
//Redistributions of source code must retain the above copyright notice, this list
//of conditions and the following disclaimer.
//
//Redistributions in binary form must reproduce the above copyright notice, this
//list of conditions and the following disclaimer in the documentation and/or
//other materials provided with the distribution.
//
//Neither the name of Robert Atkinson nor the names of his contributors may be used to
//endorse or promote products derived from this software without specific prior
//written permission.
//
//NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
//LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
//THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
//FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//*/
//package org.firstinspires.ftc.team11230;
//
//import android.content.SharedPreferences;
//import android.preference.PreferenceManager;
//
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DigitalChannelController;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.HashMap;
//
//import club.elektrakatz11230.functions.FileLogger;
//
//import static java.lang.Math.PI;
//
///**
// *   The desired path in this is to start on the edge of the tile next to corner:
// *   - Drive forward for 10 inches
// *   - Spin right for 12 Inches
// *   - Drive forward for 30 inches
// *   - Spin right for 12 Inches
// *   - Drive forward for 30 inches
// *
// *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
// *  that performs the actual movement.
// *  This methods assumes that each movement is relative to the last stopping place.
// *  There are other ways to perform encoder based moves, but this method is probably the simplest.
// *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
// */
//
//@Autonomous(name="ElektraKatz: Auto Drive to Corner", group="ElektraKatz")
//@Disabled
//class ElektraKatzAutoDriveByEncoder extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    private static final String TAG = "ElektraKatzAutoDriveByEncoder";
//
//    private final HardwareElektraKatz     robot   = new HardwareElektraKatz();   // Use ElektraKatz hardware
//    private final ElapsedTime     runtime = new ElapsedTime();
//
//    private static final double     COUNTS_PER_MOTOR_REV    = 1680 ;
//    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
//    private static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    private static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * (PI));
//    private static final double     DRIVE_SPEED             = 0.9;
//    private static final double     TURN_SPEED              = 0.7;
//    private HashMap<String,String>  powerTable              = new HashMap<String,String>();
//    private double mStepDistance;                       //used when decoding the step, this will indicate how far the robot is to move in inches
//    private int mStartPositionLeft1;                    //Left Motor 1  - start position of the robot in inches, starts from 0 to the end
//    private int mStartPositionLeft2;                    //Left Motor 2  - start position of the robot in inches, starts from 0 to the end
//    private int mStartPositionRight1;                   //Right Motor 1 - start position of the robot in inches, starts from 0 to the end
//    private int mStartPositionRight2;                   //Right Motor 2 - start position of the robot in inches, starts from 0 to the end
//    private int mStepLeftTarget1;                       //Left Motor 1   - encoder target position
//    private int mStepLeftTarget2;                       //Left Motor 2   - encoder target position
//    private int mStepRightTarget1;                      //Right Motor 1  - encoder target position
//    private int mStepRightTarget2;                      //Right Motor 2  - encoder target position
//
//    private int turnDirection=1;
//
//    SharedPreferences sharedPreferences;
//
//    //set up the CDIM
//    private DeviceInterfaceModule dim;                                  // Device Object
//    private DigitalChannel digIn;                                       // Device Object
//    private DigitalChannel digOut;                                      // Device Object
//    private boolean cdimError = false;
//
//    //set up Gyro variables
//    private boolean gyroError = false;
//    private ModernRoboticsI2cGyro gyro;                 // Hardware Device Object
//    private int gyroXVal, gyroYVal, yroZVal = 0;        // Gyro rate Values
//    private int gyroHeading = 0;                        // Gyro integrated heading
//    private int gyroAngleZ = 0;
//    private boolean gyroLastResetState = false;
//    private boolean gyroCurResetState  = false;
//
//    //set up the variables for file logger and what level of debug we will log info at
//    private FileLogger fileLogger;
//    private int debug = 3;
//
//
//
//    private void loadPowerTableTileRunner60 ()
//    {
//        powerTable.put(String.valueOf(0.5), ".3");
//        powerTable.put(String.valueOf(1), ".3");
//        powerTable.put(String.valueOf(2), ".4");
//        powerTable.put(String.valueOf(4), ".5");
//        powerTable.put(String.valueOf(6), ".6");
//        powerTable.put(String.valueOf(8), ".7");
//        powerTable.put(String.valueOf(10), ".8");
//        powerTable.put(String.valueOf(12), ".9");
//        powerTable.put(String.valueOf(15), "1.0");
//    }
//
//
//
//    @Override
//    public void runOpMode() {
//
//        /*
//         * Initialize the drive system variables.
//         * The init() method of the hardware class does all the work here
//         */
//        robot.init(hardwareMap);
//        loadPowerTableTileRunner60();
//        String allianceColor = sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Color", "null");
//        int delay = Integer.parseInt(sharedPreferences.getString("com.qualcomm.ftcrobotcontroller.Autonomous.Delay", "null"));
//        sharedPreferences = PreferenceManager.getDefaultSharedPreferences(hardwareMap.appContext);
//
//        if (debug >= 1)
//        {
//            fileLogger = new FileLogger(runtime);
//            fileLogger.open();
//            fileLogger.write("Time,SysMS,Thread,Event,Desc");
//            fileLogger.writeEvent(TAG, "Log Started");
//            runtime.reset();
//            telemetry.addData("FileLogger: ", runtime.toString());
//            telemetry.addData("FileLogger Op Out File: ", fileLogger.getFilename());
//        }
//
//        if (debug >= 1)
//        {
//            fileLogger.writeEvent(TAG, "Alliance Colour   " +  allianceColor);
////            fileLogger.writeEvent(TAG, "Alliance Position " +  alliancePosition);
//            fileLogger.writeEvent(TAG, "Alliance Delay    " +  delay);
////            fileLogger.writeEvent(TAG, "Alliance Beacons  " +  numBeacons);
////            fileLogger.writeEvent(TAG, "Robot Config      " +  robotConfig);
//        }
//
//        //don't crash the program if the GRYO is faulty, just bypass it
//        try {
//            // get a reference to a Modern Robotics GyroSensor object.
//            gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
//            // calibrate the gyro, this takes a few seconds
//            gyro.calibrate();
//        } catch (Exception e) {
//            if (debug >= 1)
//            {
//                fileLogger.writeEvent(TAG, "Gyro Error " +  e.getMessage());
//            }
//            gyroError = true;
//        }
//
//        //don't crash the program if the CDIM is faulty, just bypass it
//        try {
//
//            // get a reference to a Modern Robotics DIM, and IO channels.
//            dim = hardwareMap.get(DeviceInterfaceModule.class, "dim");      //  Use generic form of device mapping
//            digIn = hardwareMap.get(DigitalChannel.class, "digin");         //  Use generic form of device mapping
//            digOut = hardwareMap.get(DigitalChannel.class, "digout");       //  Use generic form of device mapping
//            digIn.setMode(DigitalChannelController.Mode.INPUT);             // Set the direction of each channel
//            digOut.setMode(DigitalChannelController.Mode.OUTPUT);           // Set the direction of each channel
//            //dim.setPulseWidthPeriod();
//
//
//        } catch (Exception e) {
//            if (debug >= 1)
//            {
//                fileLogger.writeEvent(TAG, "CDIM Error " +  e.getMessage());
//            }
//            cdimError = true;
//        }
//
//
//
//        // Send telemetry message to signify robot waiting;
//        telemetry.addData("Status", "Resetting Encoders");    //
//        telemetry.update();
//
//        robot.leftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        robot.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Path1",  "Left Starting at %7d :%7d",
//                robot.leftMotor1.getCurrentPosition(),
//                robot.leftMotor2.getCurrentPosition());
//        telemetry.addData("Path2",  "Right Starting at %7d :%7d",
//                robot.rightMotor1.getCurrentPosition(),
//                robot.rightMotor2.getCurrentPosition());
//        telemetry.update();
//
//        if (!gyroError) {
//            while (!isStopRequested() && gyro.isCalibrating()) {
//                sleep(50);
//                idle();
//            }
//        }
//
////        case(allianceColor){
//        if(allianceColor.equals("Blue")) {
//            turnDirection = -1;
//            dim.setLED(2,true);
//            dim.setLED(1,false);
//
//        } else {
//            turnDirection = 1;
//            dim.setLED(1,true);
//            dim.setLED(2,false);
//
//        }
//
//        if (debug >= 1)
//        {
//            fileLogger.writeEvent(TAG, "Init Complete");
//        }
//
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // Step through each leg of the path,
//        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED, 10, 10, 20.0);  // S1: Drive forward 10 to get off the wall
//     //   encoderDrive(TURN_SPEED, 3, -3, 10.0);  // S2: Turn -45 degrees
//     //   encoderDrive(DRIVE_SPEED, 30, 30, 20.0);  // S3: Drive forward 30 inches to get to center of corner
//     //   encoderDrive(TURN_SPEED, 6, -6, 10.0);  // S4: Turn -90 degrees
//     //   encoderDrive(DRIVE_SPEED, 30, 30, 10.0);  // S5: Drive 30 inches to get on to corner vortex
//
////        sleep(1000);     // pause for servos to move
//
//        if (debug >= 1)
//        {
//            if (fileLogger != null)
//            {
//                fileLogger.writeEvent(TAG, "Step FINISHED - FINISHED");
//                fileLogger.writeEvent(TAG, "Stopped");
//                fileLogger.close();
//                fileLogger = null;
//            }
//        }
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        idle();
//
//    }
//
//    /*
//     *  Method to perform a relative move, based on encoder counts.
//     *  Encoders are not reset as the move is based on the current position.
//     *  Move will stop if any of three conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Move runs out of time
//     *  3) Driver stops the opmode running.
//     */
//    private void encoderDrive(double speed,
//                              double leftInches, double rightInches,
//                              double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//        double mStepSpeedTemp;
//        double distanceToEndLeft1;
//        double distanceToEndLeft2;
//        double distanceToEndRight1;
//        double distanceToEndRight2;
//        double distanceToEnd;
//        double distanceFromStartLeft1;
//        double distanceFromStartLeft2;
//        double distanceFromStartRight1;
//        double distanceFromStartRight2;
//        double distanceFromStart;
//        double powerLeft1;
//        double powerLeft2;
//        double powerRight1;
//        double powerRight2;
//
//        int intLeft1MotorEncoderPosition;
//        int intLeft2MotorEncoderPosition;
//        int intRight1MotorEncoderPosition;
//        int intRight2MotorEncoderPosition;
//
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
////            newLeftTarget = robot.leftMotor1.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
////            newRightTarget = robot.rightMotor1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            mStepDistance = 0;
////
////            robot.rightMotor2.setTargetPosition(newRightTarget);
////            robot.rightMotor1.setTargetPosition(newRightTarget);
////            robot.leftMotor1.setTargetPosition(newLeftTarget);
////            robot.leftMotor2.setTargetPosition(newLeftTarget);
//
//            if (debug >= 2)
//            {
//                fileLogger.writeEvent("encoderDrive()", "mStepDistance   :- " + mStepDistance);
//            }
//
//            mStartPositionLeft1 = robot.leftMotor1.getCurrentPosition();
//            mStartPositionLeft2 = robot.leftMotor2.getCurrentPosition();
//            mStartPositionRight1 = robot.rightMotor1.getCurrentPosition();
//            mStartPositionRight2 = robot.rightMotor2.getCurrentPosition();
//
//            mStepLeftTarget1 = mStartPositionLeft1 + (int) (leftInches * COUNTS_PER_INCH);
//            mStepLeftTarget2 = mStartPositionLeft2 + (int) (leftInches * COUNTS_PER_INCH);
//            mStepRightTarget1 = mStartPositionRight1 + (int) (rightInches * COUNTS_PER_INCH);
//            mStepRightTarget2 = mStartPositionRight2 + (int) (rightInches * COUNTS_PER_INCH);
//
//            // pass target position to motor controller
//            robot.leftMotor1.setTargetPosition(mStepLeftTarget1);
//            robot.leftMotor2.setTargetPosition(mStepLeftTarget2);
//            robot.rightMotor1.setTargetPosition(mStepRightTarget1);
//            robot.rightMotor2.setTargetPosition(mStepRightTarget2);
//
//            if (debug >= 2)
//            {
//                fileLogger.writeEvent("running encoderDrive()", "mStepLeftTarget1 :- " + mStepLeftTarget1 +  " mStepLeftTarget2 :- " + mStepLeftTarget2);
//                fileLogger.writeEvent("running encoderDrive()", "mStepRightTarget1:- " + mStepRightTarget1 + " mStepRightTarget2:- " + mStepRightTarget2);
//            }
//
//            // Turn On RUN_TO_POSITION
//            robot.rightMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.rightMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.leftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
////            robot.rightMotor1.setPower(speed);
////            robot.rightMotor1.setPower(speed);
////            robot.leftMotor1.setPower(speed);
////            robot.leftMotor2.setPower(speed);
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            while (opModeIsActive() &&
//                   (robot.leftMotor1.isBusy() && robot.rightMotor1.isBusy())) {
//
//                setDriveMotorPower(speed);
//                intLeft1MotorEncoderPosition = robot.leftMotor1.getCurrentPosition();
//                intLeft2MotorEncoderPosition = robot.leftMotor2.getCurrentPosition();
//                intRight1MotorEncoderPosition = robot.rightMotor1.getCurrentPosition();
//                intRight2MotorEncoderPosition = robot.rightMotor2.getCurrentPosition();
//                powerLeft1=robot.leftMotor1.getPower();
//                powerLeft2=robot.leftMotor2.getPower();
//                powerRight1=robot.rightMotor1.getPower();
//                powerRight2=robot.rightMotor2.getPower();
//
//                //determine how close to target we are
//                distanceToEndLeft1 = (mStepLeftTarget1 - intLeft1MotorEncoderPosition) / COUNTS_PER_INCH;
//                distanceToEndLeft2 = (mStepLeftTarget2 - intLeft2MotorEncoderPosition) / COUNTS_PER_INCH;
//                distanceToEndRight1 = (mStepRightTarget1 - intRight1MotorEncoderPosition) / COUNTS_PER_INCH;
//                distanceToEndRight2 = (mStepRightTarget2 - intRight2MotorEncoderPosition) / COUNTS_PER_INCH;
//
//                //if getting close ramp down speed
//                distanceToEnd = (distanceToEndLeft1 + distanceToEndRight1 + distanceToEndLeft2 + distanceToEndRight2) / 4;
///*
//                if ((distanceFromStart <= 0.5 ) || (distanceToEnd <= 0.5 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(0.5)));
//                }
//                else if ((distanceFromStart <= 1 ) || (distanceToEnd <= 1 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(1)));
//                }
//                else if ((distanceFromStart <= 2 ) || (distanceToEnd <= 2 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(2)));
//                }
//                else if ((distanceFromStart <= 4 ) || (distanceToEnd <= 4 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(4)));
//                }
//                else if ((distanceFromStart <= 6 ) || (distanceToEnd <= 6 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(6)));
//                }
//                else if ((distanceFromStart <= 8 ) || (distanceToEnd <= 8 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(8)));
//                }
//                else if ((distanceFromStart <= 10 ) || (distanceToEnd <= 10 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(10)));
//                }
//                else if ((distanceFromStart <= 12 ) || (distanceToEnd <= 12 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(12)));
//                }
//                else if ((distanceFromStart <= 15 ) || (distanceToEnd <= 15 ))
//                {
//                    mStepSpeedTemp = Double.valueOf(powerTable.get(String.valueOf(15)));
//                }*/
//
//                if (debug >= 3)
//                {
//                    fileLogger.writeEvent("running encoderDrive()", "Encoder counts per inch = " + COUNTS_PER_INCH + " Power Level Set " + speed + " Power L1, L2, R1, R2" + powerLeft1 + ", " + powerLeft2 + ", " + powerRight1 + ", " + powerRight2 + " Running to target  L1, L2, R1, R2  " + mStepLeftTarget1 + ", " + mStepLeftTarget2 + ", " + mStepRightTarget1 + ",  " + mStepRightTarget2 + ", " + " Running at position L1 " + intLeft1MotorEncoderPosition + " L2 " + intLeft2MotorEncoderPosition + " R1 " + intRight1MotorEncoderPosition + " R2 " + intRight2MotorEncoderPosition);
//                }
//
//                // Display it for the driver.
//                telemetry.addData("Left 1:2",  "Running to %7d :%7d", mStepLeftTarget1,  mStepLeftTarget2);
//                telemetry.addData("LeftPos 1:2",  "Running at %7d :%7d",
//                        robot.leftMotor1.getCurrentPosition(),
//                        robot.leftMotor2.getCurrentPosition());
////                telemetry.addData("Left Power",  "Power is %3f :%3f",
////                        robot.leftMotor1.getPowerFloat(),
////                        robot.leftMotor2.getPowerFloat());
//                telemetry.addData("Left 1:2",  "Running to %7d :%7d", mStepRightTarget1,  mStepRightTarget2);
//                telemetry.addData("RightPos 1:2",  "Running at %7d :%7d",
//                        robot.rightMotor1.getCurrentPosition(),
//                        robot.rightMotor2.getCurrentPosition());
////                telemetry.addData("Right Power",  "Power is %3f :%3f",
////                        robot.rightMotor1.getPowerFloat(),
////                        robot.rightMotor2.getPowerFloat());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            setDriveMotorPower(0);
//
//            if (debug >= 2)
//            {
//                fileLogger.writeEvent("running encoderDrive()", "Complete         ");
//            }
//
////            robot.leftMotor1.setPower(0);
////            robot.leftMotor2.setPower(0);
////            robot.rightMotor1.setPower(0);
////            robot.rightMotor2.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            robot.leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            //  sleep(250);   // optional pause after each move
//        }
//    }
//    //set the drive motors power, both left and right
//    private void setDriveMotorPower (double power) {
//        setDriveRightMotorPower(power);
//        setDriveLeftMotorPower(power);
//    }
//
//    //set the right drive motors power
//    private void setDriveRightMotorPower (double power) {
//        robot.rightMotor1.setPower(power);
//        robot.rightMotor2.setPower(power);
//    }
//
//    //set the left motors drive power
//    private void setDriveLeftMotorPower (double power) {
//        robot.leftMotor1.setPower(power);
//        robot.leftMotor2.setPower(power);
//    }
//
//}
