package org.ftcteam5206.utils.Button;

import org.ftcteam5206.utils.Button.Button;

/**
 * Created by Dev on 12/17/2016.
 */

public class Buttons {
    public Button LEFT_STICK_BUTTON;
    public Button RIGHT_STICK_BUTTON;
    public Button DPAD_UP ;
    public Button DPAD_DOWN ;
    public Button DPAD_LEFT ;
    public Button DPAD_RIGHT ;
    public Button A;
    public Button B;
    public Button X;
    public Button Y;
    public Button LOGITECH;
    public Button START;
    public Button BACK;
    public Button LEFT_BUMPER;
    public Button RIGHT_BUMPER;
    public Button LEFT_TRIGGER;
    public Button RIGHT_TRIGGER;
    public Buttons() {
        LEFT_STICK_BUTTON = new Button(15, false);
        RIGHT_STICK_BUTTON = new Button(14, false);
        DPAD_UP = new Button(13, false);
        DPAD_DOWN  = new Button(12, false);
        DPAD_LEFT = new Button (11, false);
        DPAD_RIGHT  = new Button(10, false);
        A  = new Button(9, false);
        B  = new Button(8, false);
        X  = new Button(7, false);
        Y  = new Button(6, false);
        LOGITECH  = new Button(5, false);
        START  = new Button(4, false);
        BACK  = new Button(3, false);
        LEFT_BUMPER  = new Button(2, false);
        RIGHT_BUMPER  = new Button(1, false);
        LEFT_TRIGGER = new Button(16, false);//Assigned 16 to LT
        RIGHT_TRIGGER = new Button(17, false);//Assigned 17 to RT
    }
}
