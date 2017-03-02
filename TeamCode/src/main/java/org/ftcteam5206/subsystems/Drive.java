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
        return -Maths.radiansToDegrees(imu.getAngularOrientation().firstAngle - imu.getAngularVelocity().xRotationRate*milliseconds/1000.0);
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
        driveDist = inches;
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

        double leftDistanceTangentialError = kinematics.getX() - Math.abs(getLeftDriveEncoderTicks())*RobotConstants.driveTickToDist;
        double rightDistanceTangentialError = kinematics.getX() - Math.abs(getRightDriveEncoderTicks()*RobotConstants.driveTickToDist);
        double leftDistanceNormalError;
        double rightDistanceNormalError;

        double power = kv*kinematics.getY() + ka*kinematics.getZ() + kpDrive*leftDistanceTangentialError;
        leftDrive.setPower(power);
        rightDrive.setPower(power);
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

    public void plannedTurn(double degrees){
        Log.d("autodrive", "Starting Planned Turn");
        turnPath = new PlannedPath(Maths.degreeToRadians(Math.abs(degrees))*RobotConstants.driveBaseRadius, 30, 60);
        plannedTurnAngle = getRobotYaw() + degrees;
        absTurn(plannedTurnAngle);
        turnTime = OpModeTime.seconds();
    }

    //TODO: Tune these
    public final double kvTurn = 1/40.0;
    public final double kaTurn = 1/(4*RobotConstants.maxDriveAcceleration);
    public final double kpTurn = 1/400.0;
    public final double kiTurn = 1/100000.0;
    public final double kdTurn = -1/78000.0;
    double errorlast, timelast;
    double maxError;
    public void plannedTurnUpdate(){
        double deltaTime = OpModeTime.seconds() - turnTime;
        //double error = -Maths.smallestSignedAngle(plannedTurnAngle, getRobotYaw());//This is the full error as opposed to the correct model
        vector3d kinematics = turnPath.getData(deltaTime);
        double ffwdAngle = Maths.radiansToDegrees(kinematics.getX()/RobotConstants.driveBaseRadius);//This is the angle we expect to be at.
        double error =  -Maths.smallestSignedAngle(ffwdAngle, getRobotYaw());//The difference between where we expect to be and where we are.
        if(error > maxError) maxError = error;
        Log.d("autodrive", "Tracking within " + error + " of FFWD:" + ffwdAngle);
        double integralError = errorlast + error;
        errorlast = integralError;
        double derivativeError = (error - errorlast)/deltaTime;
        Log.d("autodrive", "FFWD Comps ," + kinematics.getX() + "," + kinematics.getY() + "," + kinematics.getZ());

        double pwm = kvTurn*kinematics.getY() + kaTurn*kinematics.getZ() + kpTurn*error + kiTurn*integralError + kdTurn*derivativeError;

        Log.d("autodrive", "PWM components " + kvTurn*kinematics.getY() + ", " + kaTurn*kinematics.getZ()  + ", " + kpTurn*error + ", " + kiTurn*integralError + ", " + kdTurn*derivativeError + " = " + pwm);
        if(Math.signum(processedTargetAngle) == 1.0){ //Turn Right
            leftDrive.setPower(pwm);
            rightDrive.setPower(-pwm);
            return;
        }
        leftDrive.setPower(-pwm);
        rightDrive.setPower(pwm);
    }

    double turnStartTime;
//    public void plannedTurn(double degrees){
//        turnStartTime = OpModeTime.seconds();
//        targetAngle = getRobotYaw() + degrees;
//        targetAngle = wrap360(targetAngle);
//        absTurn(targetAngle);
//        Log.d("autodrive", "Planned Turn Starting. Target is " + targetAngle + " we're currently at " + getRobotYaw());
//    }

    double previousTime = 0;
    double prevError = 0;
//    public void plannedTurnUpdate() {
//        double kP = 1/400.0;
//        double error = Maths.smallestSignedAngle(targetAngle, getRobotYaw());
//        /*double kD = 1/1000.0;
//        double tChange = OpModeTime.seconds() - previousTime;
//        tChange /= 1e9;
//        previousTime = OpModeTime.seconds();
//        double error = Maths.smallestSignedAngle(targetAngle, getRobotYaw());
//        double derivative = (error - prevError) / tChange;
//        Log.d("autodrive", "Target angle: " + targetAngle);
//        Log.d("autodrive", "Robot angle: " + getRobotAngle());
//        Log.d("autodrive", "Error: " + error);
//        prevError = error;
//        leftDrive.setPower(-kP*error);
//        rightDrive.setPower(kP*error);*/
//        if(Math.signum(processedTargetAngle) == 1.0){ //Turn Right
//            leftDrive.setPower(0.3);
//            rightDrive.setPower(-0.3);
//            return;
//        }
//        leftDrive.setPower(-0.3);
//        rightDrive.setPower(0.3);
//        //Log.d("autodrive", leftDrive.getPower() + " " + rightDrive.getPower());
//        //Log.d("autodrive", "Target angle: " + targetAngle);
//    }
    public boolean plannedTurnChecker(){
        Log.d("autodrive", getRobotYaw() + ", "  + Maths.smallestSignedAngle(getRobotYaw(), targetAngle));
        boolean isAboutEqual = Math.abs(Maths.smallestSignedAngle(getRobotYaw(), targetAngle)) < 0.5 && Math.abs(leftDrive.getPower()) < 0.15 ;//FIXME: Bad stop requirement
        //isAboutEqual = OpModeTime.seconds() - turnTime > turnPath.target_time;
        if(isAboutEqual){
            stop();
            Log.d("autodrive", "Max Error from FFWD was " + maxError);
        }
        return !isAboutEqual;
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
