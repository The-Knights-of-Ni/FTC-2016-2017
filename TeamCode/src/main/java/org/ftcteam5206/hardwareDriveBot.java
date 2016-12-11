package org.ftcteam5206;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class hardwareDriveBot {
    //DC Motors
    public DcMotor lfMotor = null;
    public DcMotor rfMotor = null;
    public DcMotor lbMotor = null;
    public DcMotor rbMotor = null;

    //Servos
    public Servo buttonPusher = null;

    //sensors
    public OpticalDistanceSensor odsSensor;
    public ColorSensor colorSensor;

    //hardware map
    HardwareMap hwMap = null;

    //private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public hardwareDriveBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors clockwise starting with right front
        rfMotor = hwMap.dcMotor.get("motor1");
        rbMotor = hwMap.dcMotor.get("motor2");
        lbMotor = hwMap.dcMotor.get("motor3");
        lfMotor = hwMap.dcMotor.get("motor4");

        //Define Servos
        //buttonPusher = hwMap.servo.get("servo");

        //Define Sensors
        //odsSensor = hwMap.opticalDistanceSensor.get("ods");
        //colorSensor = hwMap.colorSensor.get("color");

        //reverse selected motors
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        rfMotor.setDirection(DcMotor.Direction.FORWARD);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        rbMotor.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        rbMotor.setPower(0);
        lbMotor.setPower(0);

        //buttonPusher.setPosition(0.1);

        // Set all motors to run with encoders.
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

