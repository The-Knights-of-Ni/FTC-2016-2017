package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Launcher Object
 * Handles shots and lines up
 * Create one of these in each opmode you want to use the launcher in.
 */

public class Launcher {
    public DcMotor launcher;
    public Servo hood;
    public ElapsedTime OpModeTime;

    public Launcher(DcMotor launcher, Servo hood, ElapsedTime OpModeTime){
        this.launcher = launcher;
        this.hood = hood;
        FlywheelController flywheel = new FlywheelController(launcher, OpModeTime);
    }
}
