package org.ftcteam5206.subsystems;
import org.ftcteam5206.subsystems.RobotConstants;
import org.ftcteam5206.utils.*;
import org.ftcteam5206.utils.vectors.*;


/**
 * Created by Dev on 11/25/2016.
 */

public class PlannedPath {
    /*
        Take path length in, as well as max velocity and max acceleration;
        Update function is called with encoder feedback, returns motor pwm
        Creates a trapezoidal motion plan for the robot to follow.
        TODO: Add stabilization based on gyro heading (return 2d vector of left/right)
         */
    final double velocityLimit;
    final double accelerationLimit;
    final double totalDistance;

    double accelerationTime;
    double distance_while_accelerating;
    double distance_while_deccelerating;
    double distance_while_cruising;
    double target_time;

    public PlannedPath(double totalDistance){
        this(totalDistance, RobotConstants.maxDriveVelocity, RobotConstants.maxDriveAcceleration);
    }

    public PlannedPath(double totalDistance, double velocityLimit, double accelerationLimit){
        this.totalDistance = totalDistance;
        this.velocityLimit = velocityLimit;
        this.accelerationLimit = accelerationLimit;
        //Dependent Variables
        accelerationTime = velocityLimit/accelerationLimit;
        distance_while_accelerating = accelerationLimit*Maths.square(accelerationTime)/2.0;//Ignoring v_0 and x_0
        distance_while_deccelerating = accelerationLimit*Maths.square(accelerationTime)/2.0;//Ignoring v_0 and x_0, I know decceleration is the wrong term
        distance_while_cruising = totalDistance - distance_while_deccelerating - distance_while_accelerating;
        //Handles triangle case
        if(distance_while_cruising > 0)
        {
            target_time = 2.0*accelerationTime+distance_while_cruising/velocityLimit;
        }
        else
        {
            accelerationTime = Math.sqrt((totalDistance/2.0)*(2.0/accelerationLimit));
            target_time = 2.0*accelerationTime;
        }
    }

    vector3d getData(double drive_time) //X = pos, Y = vel, Z = acc
    {
        vector3d result = new vector3d(0,0,0);
        if(drive_time < accelerationTime) //accelerating
        {
            result.x = 0.5*accelerationLimit*Maths.square(drive_time);
            result.y = velocityLimit*drive_time/accelerationTime;
            result.z = accelerationLimit;
        }
        else if(drive_time > target_time) //stopping
        {
            result.x = totalDistance;
            result.y = 0;
            result.z = 0;
        }
        else if(drive_time > target_time-accelerationTime) //deccelerating
        {
            result.x = totalDistance - 0.5*accelerationLimit*Maths.square(target_time-drive_time);
            result.y = velocityLimit*(target_time-drive_time)/accelerationTime;
            result.z = -accelerationLimit;
        }
        else //cruising
        {
            result.x = distance_while_accelerating + velocityLimit*(drive_time-accelerationTime);
            result.y = velocityLimit;
            result.z = 0;
        }
        return result;
    }

    //TODO: Add Angular Error for Gyro Stabilization
    //TODO: Add derivative term (requires a known dt)
    //TODO: Tune Kv, Ka, and Kpt
    final static double Kv = 1/RobotConstants.maxDriveVelocity;
    final static double Ka = 0;//TODO: Tune this.
    final static double Kpt = 0;//Feedback disabled
    public vector2d getPWM(double driveTime, double distanceTraveled, double heading){
        vector2d pwm = new vector2d(0,0);
        if(driveTime < target_time || Math.abs(distanceTraveled - totalDistance) > RobotConstants.driveDistTolerance){
            vector3d feedForwardNumbers = getData(driveTime);
            double tangentError = feedForwardNumbers.x - distanceTraveled;

            pwm.x = Kv*feedForwardNumbers.y + Ka*feedForwardNumbers.z + Kpt*tangentError;
            pwm.y = Kv*feedForwardNumbers.y + Ka*feedForwardNumbers.z + Kpt*tangentError;
        }
        return pwm;
    }

}
