package org.ftcteam5206.subsystems;

import android.graphics.Path;
import android.util.Log;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.utils.Maths;
import org.ftcteam5206.utils.vectors.quaternion;
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
        return -Maths.radiansToDegrees(imu.getAngularOrientation().firstAngle);
    }

    public double getPredictedRobotYaw(int milliseconds){
        return -Maths.radiansToDegrees(imu.getAngularOrientation().firstAngle - imu.getAngularVelocity().firstAngleRate*milliseconds/1000.0);
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
    public double getAvgDriveEncoderTicks(){ return (Math.abs(getLeftDriveEncoderTicks()) + Math.abs(getRightDriveEncoderTicks()))/2.0;}
    //Motion Profiled Drive
    public PlannedPath drivePath;
    private double driveTime;
    public double driveBearing;
    public double driveDist;
    private boolean isBackwards = false;
    public void driveDist(double inches){
        driveDist(inches, 67);
    }
    public void driveDist(double inches, double speed){
        isBackwards = false;
        if(inches < 0){
            isBackwards = true;
            inches = Math.abs(inches);
        }
        Log.d("autodrive", "Creating Drive Dist");
        zeroDriveEncoders();
        driveDist = inches-1;
        drivePath = new PlannedPath(Math.abs(driveDist), speed, 150);
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
        double leftDistanceError = kinematics.getX() - Math.abs(getLeftDriveEncoderTicks())*RobotConstants.driveTickToDist*(isBackwards ? -1 : 1);
        double rightDistanceError = kinematics.getX() - Math.abs(getRightDriveEncoderTicks()*RobotConstants.driveTickToDist*(isBackwards ? -1 : 1));
        //Log.d("autodrive", "Drive Dist Time: " + deltaTime + " of " + drivePath.target_time*1.5);
        //Log.d("autodrive", "Motor PWM: "+ (kv*kinematics.getY() + ka*kinematics.getZ() + kpDrive*leftDistanceError));
        //double power = kv*kinematics.getY() + ka*kinematics.getZ() + kpDrive*leftDistanceError;
        double power = kv*kinematics.getY() + ka*kinematics.getZ() + kpDrive*rightDistanceError;
        leftDrive.setPower(power*(isBackwards ? -1 : 1));
        rightDrive.setPower(power*(isBackwards ? -1 : 1));
        if(leftDrive.getPower() != rightDrive.getPower());
            //Log.d("autodrive", "Left drive power: " + leftDrive.getPower() + ", right drive power: " + rightDrive.getPower());
    }

    public boolean driveDistChecker(){
        //Log.d("autodrive", "Current distance driven: " + Math.abs(getAvgDriveEncoderTicks())*RobotConstants.driveTickToDist);
        //Log.d("autodrive", "Target drive distance: " + Math.abs(driveDist));
        //Log.d("autodrive", "Time expired: " + (OpModeTime.seconds()-driveTime >= drivePath.target_time*1.5) + "");
        //Log.d("autodrive", "Encoder reached target: " + (Math.abs(getAvgDriveEncoderTicks())*RobotConstants.driveTickToDist >= Math.abs(driveDist)) + "");
        //return (OpModeTime.seconds()-driveTime < drivePath.target_time*1.5) && (Math.abs(getAvgDriveEncoderTicks())*RobotConstants.driveTickToDist < Math.abs(driveDist));
        return (OpModeTime.seconds()-driveTime < drivePath.target_time*1.5) && (Math.abs(getRightDriveEncoderTicks())*RobotConstants.driveTickToDist < Math.abs(driveDist));
    }

    //Basic Bang Bang ABS Turn
    public double processedTargetAngle;
    public double targetAngle;
    public double offsetAngle;
    private boolean turnIsClockwise = false;
    public void absTurn(double targetAngle){
        this.targetAngle = targetAngle;
        processedTargetAngle = Maths.smallestSignedAngle(getRobotYaw(), targetAngle);
        if(targetAngle >= 360) targetAngle -= 360;
    }

    public void absTurnUpdate(){
        if(turnIsClockwise) {//Turn Right
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
    /*
    public void plannedTurn(double degrees){
        turnPath = new PlannedPath(Maths.degreeToRadians(Math.abs(degrees))*RobotConstants.driveBaseRadius, 10, 150);
        plannedTurnAngle = getRobotYaw() + degrees;
        absTurn(plannedTurnAngle);
        turnTime = OpModeTime.seconds();
    }
    */
    //TODO: Tune these, add Ka
    public double kpTurn = 1/600.0;
    public double kiTurn = 0;
    double errorlast, timelast;
    /*
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
    */
    double turnStartTime;
    public void plannedTurn(double degrees){
        turnStartTime = OpModeTime.seconds();
        targetAngle = getRobotYaw() + degrees;
        targetAngle = wrap360(targetAngle);
        absTurn(targetAngle);
        turnIsClockwise = Math.signum(processedTargetAngle) == 1;
        Log.d("autodrive", "Planned Turn Starting. Target is " + targetAngle + " we're currently at " + getRobotYaw());
    }

    double previousTime = 0;
    double prevError = 0;
    public void plannedTurnUpdate() {
        double kP = 1/400.0;
        double error = Maths.smallestSignedAngle(targetAngle, getRobotYaw());
        if(turnIsClockwise){ //Turn Right
            leftDrive.setPower(0.1);
            rightDrive.setPower(-0.1);
            return;
        }
        leftDrive.setPower(-0.1);
        rightDrive.setPower(0.1);
    }

    public boolean plannedTurnChecker(){
        Log.d("autodrive", (OpModeTime.seconds() - turnStartTime) + ", " +  getRobotYaw() + ", "  + Maths.smallestSignedAngle(getRobotYaw(), targetAngle));
        //return (Math.abs(Maths.smallestSignedAngle(getRobotYaw(),targetAngle)) > 5);
        return Maths.smallestSignedAngle(getRobotYaw(),targetAngle) > 0;
        //return turnIsClockwise ? Maths.smallestSignedAngle(getRobotYaw(), targetAngle) > 0 : Maths.smallestSignedAngle(getRobotYaw(), targetAngle) < 0;
    }

    public void complexTurn(double degrees){
        returnedOnce = false;
        plannedTurn(degrees);
    }

    boolean returnedOnce = false;
    public boolean createSecondTurn = false;
    double secondTurnStartTime;
    public boolean complexTurnChecker(){
        if(!plannedTurnChecker() && !returnedOnce){
            Log.d("autodrive", "Finished first turn");
            stop();
            returnedOnce = true;
            secondTurnStartTime = OpModeTime.seconds();
            createSecondTurn = true;
        }
        if (createSecondTurn) { //In between first and second turns
            if (OpModeTime.seconds() - secondTurnStartTime > 5) { //0.5 second wait is over
                Log.d("autodrive", "Starting second turn: " + getRobotYaw() + ", " + targetAngle);
                createSecondTurn = false;
                plannedTurn(targetAngle - getRobotYaw());
            } else { //0.5 second wait is not over
                Log.d("autodrive", "Waiting for 0.5 second delay");
                return true;
            }
        }
        if(returnedOnce && !plannedTurnChecker()){
            Log.d("autodrive", "Finished with second turn");
            stop();
            return false;
        }
        return true;
    }
    private double wrap360(double degrees) {
        while(degrees > 360)
            degrees -= 360;
        while(degrees < 0)
            degrees += 360;
        return degrees;
    }

    private double adjustAngle(double angle) {
        while (angle > 180)  angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    private double getRobotAngle() {
        return wrap360(getRobotYaw() - offsetAngle);
    }

    double targetDist;
    boolean isRightTurn;
    public void encoderTurn(double degrees){
        zeroDriveEncoders();
        isRightTurn = true;
        if(degrees < 0){
            degrees = Math.abs(degrees);
            isRightTurn = false;
        }
        targetDist = Maths.degreeToRadians(degrees)*RobotConstants.driveBaseRadius;
    }

    public boolean encoderTurnChecker(){
        return targetDist > getAvgDriveEncoderTicks()*RobotConstants.driveTickToDist;
    }
    final double motorPWM = 0.3;
    public void encoderTurnUpdate(){
        rightDrive.setPower((isRightTurn ? -1:1)*motorPWM);
        leftDrive.setPower((isRightTurn ? 1:-1)*motorPWM);
    }

    //TODO: Pose Tracking, 2D Motion
}
