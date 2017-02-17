package org.ftcteam5206.utils.vectors;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

public class quaternion {
    public double x,y,z,w;

    public quaternion(double x, double y, double z, double w){
        this.x = x;
        this.y = y;
        this.z = z;
        this.w = w;
    }

    public quaternion(Quaternion q){
        this.x = q.x;
        this.y = q.y;
        this.z = q.z;
        this.w = q.w;
    }
}
