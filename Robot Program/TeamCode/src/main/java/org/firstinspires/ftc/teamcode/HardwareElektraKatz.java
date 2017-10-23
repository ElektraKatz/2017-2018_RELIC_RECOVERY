package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a GoofBot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motors:          "left motor1"     and     "left motor2"
 * Motor channel:  Right drive motors:          "right motor1"    and    "right Motor2"
 * Motor channel:  Manipulator drive motor:     "up arm"
 * Servo channel:  Servo to control left fork:  "actuator"
 */
public class HardwareElektraKatz
{

    /* Public OpMode members. */
    public DcMotor  leftMotor1   = null;
    public DcMotor  leftMotor2   = null;
    public DcMotor  rightMotor1  = null;
    public DcMotor  rightMotor2  = null;
    public DcMotor  liftMotor    = null;
    public Servo leftClaw    = null;
    // public Servo    rightClaw   = null;

    private static final double FORKS_STORE       =  0.0 ;
    // --Commented out by Inspection (11/19/2016 10:25 AM):public static final double FORKS_CLOSE       =  0.5 ;
    // --Commented out by Inspection (11/19/2016 10:25 AM):public static final double FORKS_OPEN        =  0.5 ;
    // --Commented out by Inspection (11/19/2016 10:25 AM):public static final double LIFT_UP_POWER     =  1.0 ;
    // --Commented out by Inspection (11/19/2016 10:25 AM):public static final double LIFT_DOWN_POWER   = -1.0 ;

    // --Commented out by Inspection (11/19/2016 10:28 AM):private final ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareElektraKatz(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map

        // Define and Initialize Motors
        leftMotor1   = ahwMap.dcMotor.get("left motor1");
        leftMotor2   = ahwMap.dcMotor.get("left motor2");
        rightMotor1  = ahwMap.dcMotor.get("right motor1");
        rightMotor2  = ahwMap.dcMotor.get("right motor2");
        liftMotor    = ahwMap.dcMotor.get("up motor");
        leftMotor1.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftMotor2.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightMotor1.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        rightMotor2.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        liftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors

        // Set all motors to zero power
        leftMotor1.setPower(0);
        leftMotor2.setPower(0);
        rightMotor1.setPower(0);
        rightMotor2.setPower(0);
        liftMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        leftClaw = ahwMap.servo.get("actuator");
        // rightClaw = hwMap.servo.get("right servo");
        leftClaw.setPosition(FORKS_STORE);
        // rightClaw.setPosition(FORKS_STORE);
    }

// --Commented out by Inspection START (11/19/2016 10:26 AM):
//    /***
//     *
//     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
//     * periodic tick.  This is used to compensate for varying processing times for each cycle.
//     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
//     *
//     * @param periodMs  Length of wait cycle in mSec.
//     * @throws InterruptedException
//     */
//    public void waitForTick(long periodMs) throws InterruptedException {
//
//        long  remaining = periodMs - (long)period.milliseconds();
//
//        // sleep for the remaining portion of the regular cycle period.
//        if (remaining > 0)
//            Thread.sleep(remaining);
//
//        // Reset the cycle clock for the next pass.
//        period.reset();
//    }
// --Commented out by Inspection STOP (11/19/2016 10:26 AM)
}

