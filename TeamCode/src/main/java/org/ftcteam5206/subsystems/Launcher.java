package org.ftcteam5206.subsystems;

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
    public DcMotor launcher;
    public Servo hood;
    public Flywheel flywheel;
    public ElapsedTime OpModeTime;

    public Launcher(DcMotor launcher, Servo hood, ElapsedTime OpModeTime){
        this.launcher = launcher;
        this.hood = hood;
        flywheel = new Flywheel(launcher, OpModeTime);
    }
}
