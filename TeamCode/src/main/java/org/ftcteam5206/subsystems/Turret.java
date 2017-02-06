package org.ftcteam5206.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.utils.Maths;

/**
 * Created by Dev on 1/30/2017.
 */

public class Turret {
    public DcMotor turret;
    public AnalogInput turretpot;
    public ElapsedTime OpModeTime;

    public double targetAngle, processedTargetAngle;

    Turret(DcMotor turret, AnalogInput turretpot, ElapsedTime OpModeTime){
        this.turret = turret;
        this.turretpot = turretpot;
        this.OpModeTime = OpModeTime;
    }
    //5V = 360 degrees on the turret
    public double getAngle() {
        double ticks = turret.getCurrentPosition();
        double smallRevs = ticks / (20*RobotConstants.neverRestPPR*RobotConstants.neverRestCPR);
        double bigRevs = smallRevs / (RobotConstants.turretRingGear / RobotConstants.turretMotorGear);
        double angle = 360 * bigRevs;
        return angle;
    }
        //TODO: Track Angle
    //TODO: FFWD Control to change angle (Overkill)
    //TODO: Encoder Tracking as well
    //TODO: Hold Position (Relative  to Robot)
    public double holdingAngle;

    public void holdTurret(){
        holdingAngle = getAngle();
    }
    public double kpTurret = 1/100.0;
    public void holdTurretUpdate(){
        if(Math.abs(getAngle() - holdingAngle) > 1){
            double error = getAngle() - holdingAngle;
            turret.setPower(kpTurret*error);
        }
    }

    public void turnThroughAngle(double degrees){
        targetAngle = getAngle() + degrees;
    }

    public void turnUpdate() {
        if(Math.signum(targetAngle-getAngle()) == 1)
            turret.setPower(.3);
        else
            turret.setPower(-.3);
    }

    public boolean turnChecker() {
        return Math.abs(getAngle() - targetAngle) > 10.0;
    }

    //TODO: Lock Position (Relative to field)
}
