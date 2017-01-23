package org.ftcteam5206.subsystems;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.utils.Maths;

/**
 * Drive Object
 * Handles Autonomous Control of Drivetrain
 * Create one of these in each opmode you want to use the drivetrain in.
 */
public class Drive {
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public ElapsedTime OpModeTime;
    public BNO055IMU imu;
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

    public Drive(DcMotor leftDrive, DcMotor rightDrive, BNO055IMU imu, ElapsedTime OpModeTime){
        this.leftDrive = leftDrive;
        this.rightDrive = rightDrive;
        this.imu = imu;
        this.OpModeTime = OpModeTime;
        this.driveState = DriveState.OPEN_LOOP;
    }

    private double offset1, offset2, offset3;
    public void zeroIMU(){
        offset1 = imu.getAngularOrientation().firstAngle;
        offset2 = imu.getAngularOrientation().secondAngle;
        offset3 = imu.getAngularOrientation().thirdAngle;
    }

    public double getRobotYaw(){
        //return imu.getAngularOrientation().firstAngle - offset1;
        return -imu.getAngularOrientation().firstAngle;
    }

    public void turnTest(){
        zeroIMU();
        double angle0 = getRobotYaw();
        double angleDelta = 90;
        while(!Maths.aboutEqual(angle0 + angleDelta, getRobotYaw(), 10)){
            leftDrive.setPower(0.5);
            rightDrive.setPower(-0.5);
        }
    }

    public double processedTargetAngle;
    double targetAngle;
    public void absTurn(double targetAngle){
        this.targetAngle = targetAngle;
        processedTargetAngle = Maths.smallestSignedAngle(getRobotYaw(), targetAngle);
    }

    public void absTurnUpdate(){
        if(Math.signum(processedTargetAngle) == 1.0) {//Turn Right
            leftDrive.setPower(0.5);
            rightDrive.setPower(-0.5);
            return;
        }
        leftDrive.setPower(-0.5);
        rightDrive.setPower(0.5);
    }

    public boolean absTurnChecker(double tolerance){
        return !Maths.aboutEqual(targetAngle, getRobotYaw(), tolerance);
    }
    //TODO: Path Planning, Turning, IMU, Pose Tracking, 2D Motion
}
