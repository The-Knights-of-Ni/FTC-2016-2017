package org.ftcteam5206.subsystems;
import org.ftcteam5206.utils.;
/**
 * Created by Dev on 11/25/2016.
 */

public class PlannedPath {
    /*
        Take path length in, as well as max velocity, max acceleration, and max jerk.
        Update function is called with encoder feedback, returns motor pwm
        TODO: Add stabilization based on gyro heading (return 2d vector of left/right)
         */
    /
    final float maxDriveVelocity = 18;
    final float maxDriveAcceleration = 10;
    final float velocityLimit;
    final float accelerationLimit;
    final float totalDistance;
    final float final_velocity;

    float acceleration_time;
    float distance_while_accelerating;
    float distance_while_deccelerating;
    float distance_while_cruising;
    float target_time;

    public PlannedPath(float totalDistance){
        this(totalDistance, maxDriveVelocity, maxDriveAcceleration, 0);
    }
    public PlannedPath(float totalDistance, float velocityLimit, float accelerationLimit){
        this(totalDistance, velocityLimit, accelerationLimit, 0);
    }
    public PlannedPath(float totalDistance, float velocityLimit, float accelerationLimit, float final_velocity){
        this.totalDistance = totalDistance;
        this.velocityLimit = velocityLimit;
        this.accelerationLimit = accelerationLimit;
        //Dependent Variables
        acceleration_time = velocityLimit/accelerationLimit;
        distance_while_accelerating = accelerationLimit*(acceleration_time*acceleration_time)/2.0;//Ignoring v_0 and x_0
        distance_while_deccelerating = accelerationLimit*square(acceleration_time)/2.0;//Ignoring v_0 and x_0, I know decceleration is the wrong term
        distance_while_cruising = total_distance_in - distance_while_deccelerating - distance_while_accelerating;
        //Handles triangle case
        if(distance_while_cruising > 0)
        {
            target_time = 2.0*acceleration_time+distance_while_cruising/max_velocity;
        }
        else
        {
            acceleration_time = sqrt((total_distance/2.0)*(2.0/max_acceleration));
            target_time = 2.0*acceleration_time;
        }
    }

    /*
    struct trapezoidalMotionProfile
    {
        #define max_robot_velocity 18 // in/s
        #define max_robot_acceleration 10 // in/s^2
        //Independent Variables
        float max_velocity;
        float max_acceleration;
        float total_distance;
        float final_velocity;
        //Dependent Variables
        float acceleration_time;
        float distance_while_accelerating;
        float distance_while_deccelerating;
        float distance_while_cruising;
        float target_time;

        trapezoidalMotionProfile(float max_velocity_in, float max_acceleration_in, float total_distance_in, float final_velocity_in)
        {
            //Independent Variables
            max_velocity = min(max_velocity_in, max_robot_velocity);
            max_acceleration = min(max_acceleration_in, max_robot_acceleration);
            total_distance = total_distance_in;
            final_velocity = min(final_velocity_in, max_robot_velocity);
            //Dependent Variables
            acceleration_time = max_velocity/max_acceleration;
            distance_while_accelerating = max_acceleration*sq(acceleration_time)/2.0;//Ignoring v_0 and x_0
            distance_while_deccelerating = max_acceleration*sq(acceleration_time)/2.0;//Ignoring v_0 and x_0, I know decceleration is the wrong term
            distance_while_cruising = total_distance_in - distance_while_deccelerating - distance_while_accelerating;
            //Handles triangle case
            if(distance_while_cruising > 0)
            {
                target_time = 2.0*acceleration_time+distance_while_cruising/max_velocity;
            }
            else
            {
                acceleration_time = sqrt((total_distance/2.0)*(2.0/max_acceleration));
                target_time = 2.0*acceleration_time;
            }

        }
        v3f getData(float drive_time)
        {
            v3f result = (v3f) {0,0,0};
            if(drive_time < acceleration_time) //accelerating
            {
                result[0] = 0.5*max_acceleration*sq(drive_time);
                result[1] = max_velocity*drive_time/acceleration_time;
                result[2] = max_acceleration;
            }
            else if(drive_time > target_time) //stopping
            {
                result[0] = total_distance;
                result[1] = 0;
                result[2] = 0;
            }
            else if(drive_time > target_time-acceleration_time) //deccelerating
            {
                result[0] = total_distance - 0.5*max_acceleration*sq(target_time-drive_time);
                result[1] = max_velocity*(target_time-drive_time)/acceleration_time;
                result[2] = -max_acceleration;
            }
            else //cruising
            {
                result[0] = distance_while_accelerating + max_velocity*(drive_time-acceleration_time);
                result[1] = max_velocity;
                result[2] = 0;
            }
            return result;
        }

    };*/

}
