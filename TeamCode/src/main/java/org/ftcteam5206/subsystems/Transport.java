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
    public Servo transportServo;
    public ElapsedTime OpModeTime;
    public boolean isRunning = false;

    public double transportSpeed = 0.5;

    public enum TransportState {
        OPEN_LOOP, HOLD_BALL, LAUNCH_POSITION
    }

    public TransportState transportState = TransportState.OPEN_LOOP;

    public Transport(DcMotor transportMotor, Servo transportServo, ElapsedTime opModeTime) {
        this.transportMotor = transportMotor;
        this.transportServo = transportServo;
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

    public void on(){
        transportMotor.setPower(0.75);
    }

    public void off(){
        transportMotor.setPower(0);
    }

    public void setServoPosition (double position) {
        transportServo.setPosition(position);
    }

    public double getServoPosition () {
        return transportServo.getPosition();
    }

    public TransportState getTransportState() {
        return transportState;
    }

    public void checkForStall () {
        if (transportMotor.getPower() == 0)
            return;

    }

}
