package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dev on 12/1/2016.
 */

public class Flywheel {
    /*
    Set launcher motor. Does not have exclusive control, might need to make it respect state machine.
    TODO: Feedforward
    TODO: Location Based Spin up
    TODO: Make this a motion planned thing to save on battery (The way DC motors work might mean this is stupid)
    Ramp up at full power, ramp down to sustain rpm
    */
    DcMotor launcher;
    ElapsedTime OpModeTime;
    public Flywheel(DcMotor launcher, ElapsedTime OpModeTime){
        this.launcher = launcher;//Hook actual motor to this class for simplicity.
        this.OpModeTime = OpModeTime;
    }

    final double kpLauncher = 1/3300;//IDK, might work.
    public double lastCallTime;
    public int lastCallTicks;

    public void spinUpToRPM(int targetRPM){
        double ticksSinceLastCall = launcher.getCurrentPosition() - lastCallTicks;//This probably doesn't work.
        double revolutions = ticksSinceLastCall/RobotConstants.launcherPPR;
        double timeSinceLastCall = OpModeTime.seconds() - lastCallTime;
        double currentRPM = revolutions/(timeSinceLastCall/60);
        double rpmError = targetRPM - currentRPM;
        launcher.setPower(kpLauncher*rpmError + launcher.getPower());//FIXME: P control,  pretty stupid.
        lastCallTicks = launcher.getCurrentPosition();//Need to wrap if this gets big
        lastCallTime = OpModeTime.seconds();
    }

    public void spinDown(){
        launcher.setPower(0);
    }
    double prevRPM;
    public double getRPM(){
        double ticksSinceLastCall = launcher.getCurrentPosition() - lastCallTicks;//This probably doesn't work.
        if(ticksSinceLastCall == 0)
            return -prevRPM;
        double revolutions = ticksSinceLastCall/RobotConstants.launcherPPR;
        double timeSinceLastCall = OpModeTime.seconds() - lastCallTime;
        double currentRPM = revolutions/(timeSinceLastCall/60);
        prevRPM = currentRPM;
        lastCallTicks = launcher.getCurrentPosition();//Need to wrap if this gets big
        lastCallTime = OpModeTime.seconds();
        return -currentRPM;
    }
}
