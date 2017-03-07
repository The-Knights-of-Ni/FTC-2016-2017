package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.ftcteam5206.utils.vectors.vector2d;

/**
 * Launcher Object
 * Handles shots and lines up
 * Create one of these in each opmode you want to use the launcher in.
 */
//TODO: Integrate Hood, Sensors, Lookup Table, etc. (basically everything)
public class Launcher {
    public DcMotor launcher, launcher2;
    private VoltageSensor voltageSensor;
    public Servo hood;
    public Flywheel flywheel;
    public ElapsedTime OpModeTime;
    private LauncherState launcherState = LauncherState.SEMI_AUTO;

    private double lastEncoderTicks;
    private double lastEncoderTime;
    private double lastRPM;

    public enum LauncherState {
        STOPPED, OPEN_LOOP, SEMI_AUTO, AUTO
    }

    public enum SemiAutoState {
        EXHAUSTING, TRANSPORTING, SPINNING_UP, FIRING, RELOADING
    }

    public Launcher(DcMotor launcher, DcMotor launcher2, VoltageSensor voltageSensor, Servo hood, ElapsedTime OpModeTime){
        this.launcher = launcher;
        this.launcher2 = launcher2;
        this.voltageSensor = voltageSensor;
        this.hood = hood;
        this.OpModeTime = OpModeTime;
        flywheel = new Flywheel(launcher, OpModeTime);

        lastEncoderTicks = launcher.getCurrentPosition();
        lastEncoderTime = OpModeTime.seconds();
        lastRPM = 0;
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

    public double getRPM() {
        double currentTime = OpModeTime.seconds();
        if (currentTime - lastEncoderTime < 0.1)
            return lastRPM;
        double currentEncoderTicks = -launcher.getCurrentPosition();
        double currentRPM = ((currentEncoderTicks - lastEncoderTicks)/RobotConstants.launcherPPR) / ((currentTime-lastEncoderTime)/60);
        lastEncoderTime = currentTime;
        lastEncoderTicks = currentEncoderTicks;
        lastRPM = currentRPM;
        return lastRPM;
    }

    double lastPower = 0;
    public double error = 0;
    public void setRPM(double targetRPM) {
        if(OpModeTime.seconds() - lastEncoderTime < 0.1)
            return;
        double kP = 1/5400.0;
        double currentRPM = getRPM();
        if(Math.abs(targetRPM - currentRPM) < 50)
            return;
        error = targetRPM - currentRPM;
        double power = Range.clip((error*kP + lastPower), 0.0, 1.0);

        lastPower = power;
        launcher.setPower(power);
        launcher2.setPower(power);
    }

    public void spinUp() {
        /*
        if (voltageSensor.getVoltage() < 12.5)
            launcher.setPower(1);
        else
            launcher.setPower(12/voltageSensor.getVoltage());
        */
        setPower(1);
    }

}
