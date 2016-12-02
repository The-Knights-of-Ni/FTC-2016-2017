package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Transport Object
 * Handles the timing of each servo in the transport, as well as sensors
 * Create one of these in each opmode you want to use the transport in.
 */

public class Transport {
    public Servo leftTransport;
    public Servo rightTransport;
    public Servo transportIn;
    public Servo transportOut;
    public ElapsedTime OpModeTime;

    public Transport(Servo leftTransport, Servo rightTransport, Servo transportIn, Servo transportOut, ElapsedTime opModeTime) {
        this.leftTransport = leftTransport;
        this.rightTransport = rightTransport;
        this.transportIn = transportIn;
        this.transportOut = transportOut;
        OpModeTime = opModeTime;
    }
}
