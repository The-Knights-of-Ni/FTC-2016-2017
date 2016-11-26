package org.ftcteam5206.utils;
/**
 * Created by Cat on 10/30/2016.
 */

import java.lang.Math;
import org.ftcteam5206.utils.vectors.*;

public class Maths {
    final static double pi = 3.1415926535897932384626433832795;
    final static double e_constant = 2.71828182845904523536;

    public static double square(double n) {
        return n*n;
    }
    public static double cube(double n) {
        return n*n*n;
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

}
