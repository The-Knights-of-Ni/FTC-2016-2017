package org.ftcteam5206.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Dev on 1/30/2017.
 */

public class Turret {
    DcMotor turret;
    ElapsedTime OpModeTime;

    Turret(DcMotor turret, ElapsedTime OpModeTime){
        this.turret = turret;
        this.OpModeTime = OpModeTime;
    }
}
