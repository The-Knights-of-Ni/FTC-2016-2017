package org.ftcteam5206.utils;

import org.ftcteam5206.utils.vectors.vector2d;
import static org.ftcteam5206.utils.Maths.*;

/**
 * Created by Dev on 1/20/2017.
 */

public class JoystickSmoother {
    public static vector2d deadzone(vector2d vector){
        vector2d output = new vector2d();
        output.x = vector.x;
        output.y = vector.y;
        if(Math.abs(vector.x) < 0.1)
            output.x = 0;
        if(Math.abs(vector.y) < 0.1)
            output.y = 0;
        return  output;
    }

    static final double smoothConstant254 = 0.4;
    public static vector2d smoothJoystick254Style(vector2d stick){
        stick = deadzone(stick);
        vector2d smoothed = new vector2d();
        double raw_x = clamp(stick.x, -1, 1);//Clamp between -1 and 1
        double raw_y = clamp(stick.y, -1, 1);
        smoothed.x = Math.sin((pi * smoothConstant254 * raw_x / 2.0) / (pi / 2.0)); //Sin wave: https://www.desmos.com/calculator/4hd9ovg7el
        smoothed.y = Math.sin((pi * smoothConstant254 * raw_y / 2.0) / (pi / 2.0));
        return smoothed; //Give back smooth x and y values
    }

    public static vector2d smoothJoysticksBezierStyle(vector2d stick){
        stick = deadzone(stick);
        vector2d smoothed = new vector2d();
        double raw_x = clamp(stick.x, -1, 1);//Clamp between -1 and 1
        double raw_y = clamp(stick.y, -1, 1);
        smoothed.x = quadraticBezier(raw_x, 0, 0.8, 1);
        smoothed.y = quadraticBezier(raw_y, 0, 0.15, 1);
        smoothed.x *= Math.signum(raw_x);
        smoothed.y *= Math.signum(raw_y);
        return smoothed;
    }



}
