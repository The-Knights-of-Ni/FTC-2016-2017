package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dev on 1/30/2017.
 */

public class Turret {
    DcMotor turret;
    AnalogInput turretpot;
    ElapsedTime OpModeTime;

    Turret(DcMotor turret, AnalogInput turretpot, ElapsedTime OpModeTime){
        this.turret = turret;
        this.turretpot = turretpot;
        this.OpModeTime = OpModeTime;
    }
    //5V = 360 degrees on the turret
    public double getAngle() {
        return turretpot.getVoltage() * RobotConstants.turretPotGear / RobotConstants.turretRingGear * 36.0/5.0;
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


    //TODO: Lock Position (Relative to field)
}
