package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Transport Object
 * Handles the timing of each servo in the transport, as well as sensors
 * Create one of these in each opmode you want to use the transport in.
 */

public class Transport {
    public DcMotor transportMotor;
    public ElapsedTime OpModeTime;
    public boolean isRunning = false;

    public double transportSpeed = 0.5;

    public Transport(DcMotor transportMotor, ElapsedTime opModeTime) {
        this.transportMotor = transportMotor;
        OpModeTime = opModeTime;
    }

    public void toggleTransport(){
        if(isRunning){
            transportMotor.setPower(0);
        } else {
            transportMotor.setPower(transportSpeed);
        }
        isRunning = !isRunning;
    }
}
