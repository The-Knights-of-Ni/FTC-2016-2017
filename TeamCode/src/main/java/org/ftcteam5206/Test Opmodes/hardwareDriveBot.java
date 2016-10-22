package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class hardwareDriveBot {
    /* Public OpMode members. */
    public DcMotor lfMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor lbMotor = null;
    public DcMotor rbMotor = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public hardwareDriveBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        lfMotor = hwMap.dcMotor.get("motor3");
        rfMotor = hwMap.dcMotor.get("motor1");
        lbMotor = hwMap.dcMotor.get("motor2");
        rbMotor = hwMap.dcMotor.get("motor4");

        lfMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rfMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        lbMotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rbMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        // Set all motors to zero power
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        rbMotor.setPower(0);
        lbMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        lfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}

