package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.ftcteam5206.subsystems.Cap.CapState.STOPPED;

/**
 * Cap Object
 * Handles the timing of all cap servos and sensors
 * Create one of these in each opmode you want to use the cap mech in.
 */
//TODO: Add in timing and releases
public class Cap {
    public Servo forkReleaseLeft;
    public Servo forkReleaseRight;
    public Servo clasp;
    public DcMotor capMotor;
    public ElapsedTime OpModeTime;

    public enum CapState {
        STOPPED, FORKS_DOWN, CLASP_ON, LIFTING, LIFTED
    }

    public CapState capState = STOPPED;

    public Cap(Servo forkReleaseLeft, Servo forkReleaseRight, Servo clasp, DcMotor capMotor, ElapsedTime OpModeTime) {
        this.forkReleaseLeft = forkReleaseLeft;
        this.forkReleaseRight = forkReleaseRight;
        this.clasp = clasp;
        this.capMotor = capMotor;
        this.OpModeTime = OpModeTime;
    }

    //TODO: Program servos
    public void releaseForks() {
        forkReleaseRight.setPosition(1);
        forkReleaseRight.setPosition(1);
    }

    public void deployClasp() {
        clasp.setPosition(1);
    }

    public void liftCap() {

    }
}