package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Beacon Pusher Object
 * Handles the servo and the vision system
 * Create one of these in each opmode you want to use the beacon/beacon vision in.
 */

public class BeaconPusher {
    public Servo beaconPusher;
    public ElapsedTime OpModeTime;

    public BeaconPusher(Servo beaconPusher, ElapsedTime opModeTime) {
        this.beaconPusher = beaconPusher;
        OpModeTime = opModeTime;
    }
}
