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
    private DriveState driveState;


    public enum DriveState {
        STOPPED, OPEN_LOOP, AUTO
    }

    public DriveState getDriveState(){
        return driveState;
    }
    //TODO: Add safeties in here to make sure we don't go to a bad state.
    public void setDriveState(DriveState state){
        driveState = state;
    }

    public void stop(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        driveState = DriveState.STOPPED;
    }

    public Drive(DcMotor leftDrive, DcMotor rightDrive, ElapsedTime OpModeTime){
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.OpModeTime = OpModeTime;
        this.driveState = DriveState.OPEN_LOOP;
    }
    //TODO: Path Planning, Turning, IMU, Pose Tracking, 2D Motion
}
