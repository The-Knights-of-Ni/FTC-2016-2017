package org.ftcteam5206.utils;
/**
 * Created by Cat on 10/30/2016.
 */

import java.lang.Math;
import org.ftcteam5206.utils.vectors.*;

public class Maths {
    public final static double pi = 3.1415926535897932384626433832795;
    public final static double e_constant = 2.71828182845904523536;

    public static double square(double n) {
        return n*n;
    }
    public static double cube(double n) {
        return n*n*n;
    }

    public static double clamp(double n, double lower, double upper){
        if(n < lower) return lower;
        if(n > upper) return upper;
        return n;
    }

    public static double quadraticBezier(double t, double p0, double p1, double p2)//https://www.desmos.com/calculator/4ut4smjf8g
    {
        if(t < 0) t *= -1;//Produces unexpected results if you pass negatives
        return (1-t)*(1-t)*p0 + 2*t*(1-t)*p1 + t*t*p2;
    }

    public static boolean aboutEqual(double a, double b, double tolerance){
        return (b <= a + tolerance/2 && b >= a - tolerance/2);
    }

    public static double smallestSignedAngle(double a, double b){
        double output = b - a;
        if(output > 180) output -= 360;
        if(output < -180) output += 360;
        return output;
    }
    /* Angle operations */
    public static double degreeToRadians(double degrees) {
        return degrees*2.0*pi/360.0;
    }

    public static double radiansToDegrees(double radians) {
        return radians*360.0/(2.0*pi);
    }

    public static double getDifferenceRadians(double from, double to) {
        return to-from;
    }

    public static double signedNormalizedDegrees(double degrees) {
        // NOTE: this can return both -180 and +180, the sign will be unchanged for these angles
        return degrees - 360 * Math.floor(degrees/360+0.5);
    }

    public static double boundAngleNegPiToPiRadians(double angle) {
        // Naive algorithm
        while (angle >= pi) {
            angle -= 2.0 * pi;
        }
        while (angle < -pi) {
            angle += 2.0 * pi;
        }
        return angle;
    }

    public static double boundAngle0to2PiRadians(double angle) {
        // Naive algorithm
        while (angle >= 2.0 * pi) {
            angle -= 2.0 * pi;
        }
        while (angle < 0.0) {
            angle += 2.0 * pi;
        }
        return angle;
    }
    /* End of angle operations */

    /* 2D Vector Operations */
    public static vector2d addV2(vector2d v1, vector2d v2) {
        double x = v1.x + v2.x;
        double y = v1.y + v2.y;
        return new vector2d(x, y);
    }

    public static vector2d subtractV2(vector2d v1, vector2d v2) {
        double x = v1.x - v2.x;
        double y = v1.y - v2.y;
        return new vector2d(x, y);
    }

    public static vector2d multiplyV2byScalar(vector2d v, double a) {
        double x = v.x * a;
        double y = v.y * a;
        return new vector2d(x, y);
    }

    public static vector2d multiplyV2byScalar(double a, vector2d v) {
        double x = v.x * a;
        double y = v.y * a;
        return new vector2d(x, y);
    }
    /* End of 2D vector operations */

    public static vector3d quaternionToEuler(quaternion q){
        double ysqr = q.y * q.y;
        vector3d output = new vector3d(0,0,0);
        // roll (x-axis rotation)
        double t0 = +2.0 * (q.w * q.x + q.y * q.z);
        double t1 = +1.0 - 2.0 * (q.x * q.x + ysqr);
        output.x = Math.atan2(t0, t1);

        // pitch (y-axis rotation)
        double t2 = +2.0 * (q.w * q.y - q.z * q.x);
        t2 = t2 > 1.0 ? 1.0 : t2;
        t2 = t2 < -1.0 ? -1.0 : t2;
        output.y = Math.asin(t2);

        // yaw (z-axis rotation)
        double t3 = +2.0 * (q.w * q.z + q.x * q.y);
        double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
        output.z = Math.atan2(t3, t4);
        return output;
    }

    public static double lightweightQuatToYaw(quaternion q){
        /*double ysqr = q.y * q.y;
        double t3 = +2.0 * (q.w * q.z + q.x * q.y);
        double t4 = +1.0 - 2.0 * (ysqr + q.z * q.z);
        return Math.atan2(t3, t4);*/
        return Math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z) ) * 180.0 / Math.PI;
    }
}
