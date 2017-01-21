package org.ftcteam5206;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.subsystems.RobotConstants;

/**
 * Created by Dev on 11/25/2016.
 */

public class KingArthur {
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor launcher = null;
    public DcMotor intakeTransport = null;
    public DcMotor redacted = null;

    public Servo beaconPusher = null;
    public Servo forkRelease = null;
    public Servo clasp = null;
    public Servo hood = null;
    public Servo capRelease = null;

    HardwareMap hwMap = null;
    public KingArthur(){}

    public void init(HardwareMap hwMap){
        this.hwMap = hwMap;

        //Init DCs
        leftDrive   = hwMap.dcMotor.get("ldrive");
        rightDrive  = hwMap.dcMotor.get("rdrive");
        launcher    = hwMap.dcMotor.get("launcher");
        intakeTransport = hwMap.dcMotor.get("intake");
        redacted = hwMap.dcMotor.get("transport");
        //Set Directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeTransport.setDirection(DcMotor.Direction.FORWARD);
        intakeTransport.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        redacted.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        launcher.setPower(0);
        intakeTransport.setPower(0);
        redacted.setPower(0);

        // Set all motors to run in open loop.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTransport.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        redacted.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Define and initialize ALL installed servos.
        beaconPusher = hwMap.servo.get("beacon");
        forkRelease= hwMap.servo.get("fork");
        clasp = hwMap.servo.get("clasp");
        hood = hwMap.servo.get("hood");
        capRelease = hwMap.servo.get("cap");
    }

    public void resetEncoders(){
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeTransport.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        redacted.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Might need an idle/wait here.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransport.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        redacted.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    //TODO: See if we need to wait for tick

    public void imuCalibrate(){
        throw new UnsupportedOperationException();
    }
}
