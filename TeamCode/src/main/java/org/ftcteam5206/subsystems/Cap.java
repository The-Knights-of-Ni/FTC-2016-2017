package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Cap Object
 * Handles the timing of all cap servos and sensors
 * Create one of these in each opmode you want to use the cap mech in.
 */
//TODO: Add in timing and releases
public class Cap {
    public Servo forkRelease;
    public Servo clasp;
    public Servo capRelease;
    public ElapsedTime OpModeTime;


    public Cap(Servo capRelease, Servo forkRelease, Servo clasp, ElapsedTime OpModeTime) {
        this.capRelease = capRelease;
        this.forkRelease = forkRelease;
        this.clasp = clasp;
        this.OpModeTime = OpModeTime;
    }
}
