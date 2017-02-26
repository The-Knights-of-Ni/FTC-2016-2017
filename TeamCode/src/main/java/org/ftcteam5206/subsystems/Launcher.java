package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Launcher Object
 * Handles shots and lines up
 * Create one of these in each opmode you want to use the launcher in.
 */
//TODO: Integrate Hood, Sensors, Lookup Table, etc. (basically everything)
public class Launcher {
    public DcMotor launcher, launcher2;
    public Servo hood;
    public Flywheel flywheel;
    public ElapsedTime OpModeTime;
    private LauncherState launcherState = LauncherState.SEMI_AUTO;

    public enum LauncherState {
        STOPPED, OPEN_LOOP, SEMI_AUTO, AUTO
    }

    public enum SemiAutoState {
        EXHAUSTING, TRANSPORTING, SPINNING_UP, FIRING, RELOADING
    }

    public Launcher(DcMotor launcher, DcMotor launcher2, Servo hood, ElapsedTime OpModeTime){
        this.launcher = launcher;
        this.launcher2 = launcher2;
        this.hood = hood;
        this.OpModeTime = OpModeTime;
        flywheel = new Flywheel(launcher, OpModeTime);
    }

    public void layup() {
        launcher.setPower(0.5);
    }

    public void midrange(){
        launcher.setPower(0.75);
    }

    public void spinDown(){
        flywheel.spinDown();
    }

    public LauncherState getLauncherState() {
        return launcherState;
    }

    public void setLauncherState(LauncherState launcherState) {
        this.launcherState = launcherState;
    }

    public void setPower(double power) {
        launcher.setPower(power);
        launcher2.setPower(power);
    }

}
