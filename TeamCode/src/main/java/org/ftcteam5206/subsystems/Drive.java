package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Drive Object
 * Handles Autonomous Control of Drivetrain
 * Create one of these in each opmode you want to use the drivetrain in.
 */
public class Drive {
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public ElapsedTime OpModeTime;

    public Drive(DcMotor leftDrive, DcMotor rightDrive, ElapsedTime OpModeTime){
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.OpModeTime = OpModeTime;
    }
    //TODO: Path Planning, Turning, IMU, Pose Tracking, 2D Motion
}
