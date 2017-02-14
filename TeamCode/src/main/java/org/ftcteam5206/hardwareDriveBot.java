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
    public DcMotor rShooter = null;
    public DcMotor lShooter = null;

    //Servos
    public Servo intake = null;
    public Servo transport = null;
    public Servo transloader =  null;
    public Servo loader = null;

    //sensors
    public OpticalDistanceSensor odsSensor;
    public ColorSensor colorSensor;

    //hardware map
    HardwareMap hwMap = null;

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
        rShooter = hwMap.dcMotor.get("rShooter");
        lShooter = hwMap.dcMotor.get("lShooter");

        //Define Servos
        intake = hwMap.servo.get("intake");
        transport = hwMap.servo.get("tport");
        transloader = hwMap.servo.get("tload");
        loader = hwMap.servo.get("load");

        //Define Sensors
        //odsSensor = hwMap.opticalDistanceSensor.get("ods");
        //colorSensor = hwMap.colorSensor.get("color");

        //reverse selected motors
        lfMotor.setDirection(DcMotor.Direction.REVERSE);
        lbMotor.setDirection(DcMotor.Direction.REVERSE);
        rfMotor.setDirection(DcMotor.Direction.FORWARD);
        rbMotor.setDirection(DcMotor.Direction.FORWARD);
        rShooter.setDirection(DcMotor.Direction.FORWARD);
        lShooter.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        lfMotor.setPower(0);
        rfMotor.setPower(0);
        rbMotor.setPower(0);
        lbMotor.setPower(0);
        rShooter.setPower(0);
        lShooter.setPower(0);

        //set servo positions
        transport.setPosition(0.5);
        intake.setPosition(0.5);
        transloader.setPosition(0.05);
        loader.setPosition(0.5);
        
        // Set all motors to run with encoders.
        lfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}

