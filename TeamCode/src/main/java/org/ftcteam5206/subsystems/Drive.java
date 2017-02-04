package org.ftcteam5206.subsystems;

import android.util.Log;

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
    private int ldoffset, rdoffset;
    public void zeroDriveEncoders(){
        ldoffset = leftDrive.getCurrentPosition();
        rdoffset = rightDrive.getCurrentPosition();
        Log.d("autodrive", "Encoders were zeroed");
    }

    public int getLeftDriveEncoderTicks(){
        return leftDrive.getCurrentPosition()-ldoffset;
    }
    public int getRightDriveEncoderTicks() { return rightDrive.getCurrentPosition() - rdoffset;}
    public double getAvgDriveEncoderTicks(){ return (getLeftDriveEncoderTicks() + getRightDriveEncoderTicks())/2.0;}
    //Motion Profiled Drive
    public PlannedPath drivePath;
    private double driveTime;
    public double driveBearing;
    public double driveDist;
    public void driveDist(double inches){
        Log.d("autodrive", "Creating Drive Dist");
        zeroDriveEncoders();
        driveDist = inches-1;
        drivePath = new PlannedPath(Math.abs(driveDist), 67, 150);
        driveBearing = getRobotYaw();
        driveTime = OpModeTime.seconds();
    }
    //TODO: Tune these, add in gyro stabilization (tangential and normal error)
    public double kv = 1/RobotConstants.maxDriveVelocity;
    public double ka = 1/(8*RobotConstants.maxDriveAcceleration);
    public double kpDrive = 1/200.0;
    //kdDrive;
    public void driveDistUpdate(){
        double deltaTime = OpModeTime.seconds()-driveTime;
        vector3d kinematics = drivePath.getData(deltaTime);
        double leftDistanceError = kinematics.getX() - Math.abs(getLeftDriveEncoderTicks())*RobotConstants.driveTickToDist;
        double rightDistanceError = kinematics.getX() - Math.abs(getRightDriveEncoderTicks()*RobotConstants.driveTickToDist);
        Log.d("autodrive", "Drive Dist Time: " + deltaTime + " of " + drivePath.target_time*1.5);
        Log.d("autodrive", "Motor PWM: "+ (kv*kinematics.getY() + ka*kinematics.getZ() + kpDrive*leftDistanceError));
        double power = kv*kinematics.getY() + ka*kinematics.getZ() + kpDrive*leftDistanceError;
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        if(leftDrive.getPower() != rightDrive.getPower())
            Log.d("autodrive", "Left drive power: " + leftDrive.getPower() + ", right drive power: " + rightDrive.getPower());
    }

    public boolean driveDistChecker(){
        Log.d("autodrive", "Current distance driven: " + Math.abs(getAvgDriveEncoderTicks())*RobotConstants.driveTickToDist);
        //Log.d("autodrive", "Target drive distance: " + Math.abs(driveDist));
        Log.d("autodrive", "Time expired: " + (OpModeTime.seconds()-driveTime >= drivePath.target_time*1.5) + "");
        Log.d("autodrive", "Encoder reached target: " + (Math.abs(getAvgDriveEncoderTicks())*RobotConstants.driveTickToDist >= Math.abs(driveDist)) + "");
        return (OpModeTime.seconds()-driveTime < drivePath.target_time*1.5) && (Math.abs(getAvgDriveEncoderTicks())*RobotConstants.driveTickToDist < Math.abs(driveDist));
    }

    //Basic Bang Bang ABS Turn
    public double processedTargetAngle;
    double targetAngle;
    public void absTurn(double targetAngle){
        this.targetAngle = targetAngle;
        processedTargetAngle = Maths.smallestSignedAngle(getRobotYaw(), targetAngle);
        if(targetAngle >= 360) targetAngle -= 360;
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
    private double plannedTurnAngle;
    public void plannedTurn(double degrees){
        turnPath = new PlannedPath(Maths.degreeToRadians(Math.abs(degrees))*RobotConstants.driveBaseRadius, 20, 150);
        plannedTurnAngle = getRobotYaw() + degrees;
        absTurn(plannedTurnAngle);
        turnTime = OpModeTime.seconds();
    }
    //TODO: Tune these, add Ka
    public double kpTurn = 1/600.0;
    public double kiTurn = 0;
    double errorlast, timelast;
    public void plannedTurnUpdate(){
        double deltaTime = OpModeTime.seconds() - turnTime;
        double error = -Maths.smallestSignedAngle(plannedTurnAngle, getRobotYaw());
        double integralerror = errorlast + error;
        errorlast = integralerror;
        vector3d kinematics = turnPath.getData(deltaTime);
        Log.d("FFWD", "FFWD Comps ," + deltaTime + "," + kinematics.getX() + "," + kinematics.getY() + "," + kinematics.getZ());
        Log.d("autodrive", "PWM components " + kv*kinematics.getY() + ", " + ka*kinematics.getZ()  + ", " + kpTurn*error + ", " + kiTurn*integralerror + " = " + (kv*kinematics.getY() + ka*kinematics.getZ() + kpTurn*error + kiTurn*integralerror));
        if(Math.signum(processedTargetAngle) == 1.0){ //Turn Right
            leftDrive.setPower(kv*kinematics.getY() + ka*kinematics.getZ() + kpTurn*error + kiTurn*integralerror);
            rightDrive.setPower(-kv*kinematics.getY() - ka*kinematics.getZ() - kpTurn*error - kiTurn*integralerror);
            return;
        }
        leftDrive.setPower(-kv*kinematics.getY() - ka*kinematics.getZ() - kpTurn*error - kiTurn*integralerror);
        rightDrive.setPower(kv*kinematics.getY() + ka*kinematics.getZ() + kpTurn*error + kiTurn*integralerror);
    }

    public boolean plannedTurnChecker(){
        if(targetAngle >= 360)  targetAngle -= 360;
        Log.d("autodrive", OpModeTime.seconds()-turnTime + " of " + turnPath.target_time);
        Log.d("autodrive", "Robot Yaw " + getRobotYaw() + " of " + targetAngle);
        return /*(OpModeTime.seconds()-turnTime < turnPath.target_time*1.5) && */!Maths.aboutEqual(targetAngle, getRobotYaw(), 1);
    }

    void turnAbsDeg(double angle, double vIs)
    {
        if(vIs < 0)
        {
            vIs = Math.abs(vIs);
            angle = -angle;
        }

        double target_heading = angle;
        double turning_compensation = 0;

        while (Math.abs(Maths.signedNormalizedDegrees(getRobotYaw()-target_heading)) > acceptableAngleError
                || Math.abs(imu.getAngularVelocity().firstAngleRate) > 2)
        {
            float heading_error = Maths.signedNormalizedDegrees(target_heading-);
            float turning_factor = turn_kp*heading_error;

            if(left_drive_omega < 0.5 && right_drive_omega < 0.5)
            {
                turning_compensation += turn_ki*heading_error*dt;
            }
            else
            {
                turning_compensation = lerp(0.0, turning_compensation, exp(-1.0*dt));
            }
            turning_compensation = clamp(turning_compensation, -2.0, 2.0);
            turning_factor += turning_compensation;

            left_drive = -turning_factor;
            right_drive = +turning_factor;

            left_drive = clamp(left_drive, -vIs, vIs);
            right_drive = clamp(right_drive, -vIs, vIs);
            autonomousUpdate();
        }
        right_drive = 0;
        left_drive = 0;
    }

    //TODO: Pose Tracking, 2D Motion
}
