package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Intake Object
 * Handles Intake Motor and Sensors
 * Create one of these in each opmode you want to use the intake in.
 */
//TODO: Add sensors, See if an exhaust method is possible (just spit out one ball and then go back to intaking)
public class Intake {
    public DcMotor intake;
    public ElapsedTime OpModeTime;
    private IntakeState intakeState = IntakeState.OPEN_LOOP;

    public enum IntakeState {
        STOPPED, OPEN_LOOP, AUTO;
    }


    public Intake(DcMotor intake, ElapsedTime OpModeTime){
        this.intake = intake;
        this.OpModeTime = OpModeTime;
    }

    public void intakeOn(){
        intake.setPower(1);
    }

    public void intakeOff(){
        intake.setPower(0);
    }

    public void intakeReverse(){
        intake.setPower(-1);
    }

    public IntakeState getIntakeState() {
        return intakeState;
    }

    public void setIntakeState(IntakeState intakeState) {
        this.intakeState = intakeState;
    }
}
