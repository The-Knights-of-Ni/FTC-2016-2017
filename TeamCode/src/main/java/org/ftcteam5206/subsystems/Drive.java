package org.ftcteam5206.subsystems;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.utils.Maths;
import org.ftcteam5206.utils.vectors.vector3d;

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

    public double getRobotYaw(){
        return -imu.getAngularOrientation().firstAngle;
    }
    private int offset;
    public void zeroDriveEncoders(){
        offset = leftDrive.getCurrentPosition();
    }

    public int getDriveEncoderTicks(){
        return leftDrive.getCurrentPosition()-offset;
    }
    //Motion Profiled Drive
    public PlannedPath drivePath;
    private double driveTime;
    public double driveBearing;
    public double driveDist;
    public void driveDist(double inches){
        zeroDriveEncoders();
        driveDist = inches;
        drivePath = new PlannedPath(Math.abs(inches));
        driveBearing = getRobotYaw();
        driveTime = OpModeTime.seconds();
    }
    //TODO: Tune these, close loop with encoders, add in gyro stabilization (tangential and normal error)
    public double kv = 1/RobotConstants.maxDriveVelocity;
    public double ka = 1/(4*RobotConstants.maxDriveAcceleration);
    public double kpDrive, kdDrive;
    public void driveDistUpdate(){
        double deltaTime = OpModeTime.seconds()-driveTime;
        vector3d kinematics = drivePath.getData(deltaTime);
        leftDrive.setPower(kv*kinematics.getY());
        rightDrive.setPower(kv*kinematics.getY());
    }

    public boolean driveDistChecker(){
        return (OpModeTime.seconds()-driveTime >= drivePath.target_time) || (Math.abs(getDriveEncoderTicks())*RobotConstants.driveTicksToDist >= Math.abs(driveDist));
    }

    //Basic Bang Bang ABS Turn
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
    //Motion Profiled Turn
    public PlannedPath turnPath;
    private double turnTime;
    public void plannedTurn(double degrees){
        turnPath = new PlannedPath(Maths.degreeToRadians(Math.abs(degrees))*RobotConstants.driveBaseRadius);
        absTurn(getRobotYaw() + degrees);
        turnTime = OpModeTime.seconds();
    }
    //TODO: Tune these and then close loop with gyro
    public double kpTurn, kdTurn;
    public void plannedTurnUpdate(){
        double deltaTime = OpModeTime.seconds() - turnTime;
        vector3d kinematics = turnPath.getData(deltaTime);
        if(Math.signum(processedTargetAngle) == 1.0){ //Turn Right
            leftDrive.setPower(kv*kinematics.getY());
            rightDrive.setPower(-kv*kinematics.getY());
            return;
        }
        leftDrive.setPower(-kv*kinematics.getY());
        rightDrive.setPower(kv*kinematics.getY());
    }

    public boolean plannedTurnChecker(){
        return (OpModeTime.seconds()-turnTime >= turnPath.target_time) || !Maths.aboutEqual(targetAngle, getRobotYaw(), 1);
    }

    //TODO: Pose Tracking, 2D Motion
}
