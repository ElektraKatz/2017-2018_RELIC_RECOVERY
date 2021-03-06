package club.elektrakatz11230.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import club.elektrakatz11230.robotconfig.HardwareArmMotors;

/**
 * Created by Andrew Haselton on 10/15/2016 at 9:53 AM.
 */
@TeleOp(name = "Base Drive Flicker", group = "5291 Test")
@Disabled
public class BaseDriveFlicker extends OpMode{

    HardwareArmMotors attachments   = new HardwareArmMotors();   // Use base drive hardware configuration
    DcMotor flicker;


    @Override
    public void init() {
        attachments.init(hardwareMap);
        // Set all motors to zero power
        attachments.flicker.setPower(0);


        attachments.flicker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        attachments.flicker.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        if (gamepad1.a)
        {
            // eg: TETRIX = 1440 pulses
            attachments.flicker.setPower(1);
        } else {
            attachments.flicker.setPower(0);
        }



    }
}
