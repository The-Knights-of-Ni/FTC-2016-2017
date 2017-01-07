package org.ftcteam5206.utils.Button;

import java.nio.ByteBuffer;

/**
 * Handles buttons for each joystick. Allows you to implement button actions in a clean way at the
 * start of the loop.
 *
 * copied from ftc_app/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/ButtonHandler.java
 * https://github.com/The-Knights-of-Ni/ftc_app/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/Button.java
 */
public class ButtonHandler //Might want to extend gamepad instead of passing to it later. I think the opmode structure makes this annoying though.
{
    //TODO: Tap - similar to press, maybe not needed (Press that only triggers once)
    //TODO: Combination - true if all buttons pressed
    //TODO: Release - true when released
    //TODO: Trigger -> ButtonHandler option
    private int currentByte = 0; //Set these in update function
    private int previousByte = 0;//Can assume this is 0 at init.
    public float rightTriggerCurrent;
    public float rightTriggerPrevious;
    public float leftTriggerCurrent;
    public float leftTriggerPrevious;
    final int offset = 40;//Location of the ButtonHandler storage integer in Gamepad.java
    public Buttons buttons;

    public ButtonHandler() {
        this.buttons = new Buttons();
    }
    public double hairTrigger = 0.9;
    public void setHairTriggerLevel(double level){
        hairTrigger = level;
    }

    //Check the state of a button inside of the button integer.
    private boolean getState(int btn, int bits)
    {
        if(((bits >>> (btn-1)) & 1) != 0)
            return true;
        return false;
    }
    //============================ Start of actual button uses ================================

    /**
     * Assign a button to press. Leave in loop.
     * @param btn A button on the joystick.
     * @return true while the button is held, false while it's not
     */
    public boolean press(Button btn)
    {
        if(getState(btn.value, currentByte) || ((btn.value == 16) && (leftTriggerCurrent >= hairTrigger))
                || ((btn.value == 17) && (rightTriggerCurrent >= hairTrigger)))
            return true;
        return false;
    }

    /**
     * Assign a button to press. Leave in loop.
     * @param btn A button on the joystick.
     * @return true when the button is first pressed, false while it's not
     */
    public boolean singlePress(Button btn)
    {
        if(btn.value == 16) return (leftTriggerCurrent >= hairTrigger) && (leftTriggerPrevious < hairTrigger);
        if(btn.value == 17) return (rightTriggerCurrent >= hairTrigger) && (rightTriggerPrevious < hairTrigger);
        return (getState(btn.value, currentByte) == true) && (getState(btn.value, previousByte) == false);
    }

    /**
     * Assign a button to toggle. Leave in loop.
     * @param btn A button on the joystick.
     * @return true or false depending on state, button switches state.
     */
    public boolean toggle(Button btn)
    {
        if(((getState(btn.value, currentByte) == true) && (getState(btn.value, previousByte) == false))
                || ((btn.value == 16) && (leftTriggerCurrent >= hairTrigger) && (leftTriggerPrevious < hairTrigger))
                || ((btn.value == 17) && (rightTriggerCurrent >= hairTrigger) && (rightTriggerPrevious < hairTrigger))
                )//If there's a change and it's from low to high, flip it, if not, keep toggle the same.
            btn.status = !btn.status;
        return btn.status;
    }

    /**
     * Not working yet.
     * @param btn A button on the joystick.
     * @return
     */
    private boolean combinationPress(Button[] btn) //TODO: Make this a lookup table.
    {
        boolean isEqual = true;
        for(int i = 0; i < btn.length; i++){
            if(!getState(btn[i].value, currentByte))
                isEqual = false;
        }
        return isEqual;
    }

    /**
     * Not yet implemented.
     * @param btn A button on the joystick
     * @return true when the button is released, false when pressed/not pressed.
     */
    private boolean release(Button btn)
    {
        if((!getState(btn.value, currentByte)) && (getState(btn.value, currentByte) != getState(btn.value, previousByte)))
            return false;
        return true;
    }

    /**
     * Update function that refreshes the button storage integers and makes it work.
     * To implement, call object.updateButtons(gamepad#.toByteArray()) in a try/catch inside the loop.
     * @param joystick The byte array from a gamepad.
     */
    public int updateButtons(byte[] joystick, float leftTrigger, float rightTrigger) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
    {
        previousByte = currentByte;
        leftTriggerPrevious = leftTriggerCurrent;
        rightTriggerPrevious = rightTriggerCurrent;
        leftTriggerCurrent = leftTrigger;
        rightTriggerCurrent = rightTrigger;

        currentByte = ByteBuffer.wrap(joystick, 42, 4).getInt();
        return currentByte;
    }



    /*
    (Saving for EN - This is how I built the class)
    Should take a byte array from the joystick, only grab the bytes we need (the button + triggers)
    45 bytes per controller
    Testing reveals that
    At getInt(39):
    leftstick = 64
    rightstick = 32
    dpadup = 16
    dpaddown = 8
    dpadleft = 4
    dpadright = 2
    a = 1
    At getInt(38) we only see right trigger
    At getInt(40): all buttons are registered.
          1  var1.put(this.getRobocolMsgType().asByte());
          2  var1.putShort((short)42);
          1  var1.put((byte)2);
          4  var1.putInt(this.id);
          8  var1.putLong(this.timestamp).array();
          4  var1.putFloat(this.left_stick_x).array();
          4  var1.putFloat(this.left_stick_y).array();
          4  var1.putFloat(this.right_stick_x).array();
          4  var1.putFloat(this.right_stick_y).array();
            TODO: 16 + 8 + 4 + 1 + 2 = 30 bytes until triggers. (Not a todo, I just wanted the color)
          4  var1.putFloat(this.left_trigger).array();
          4  var1.putFloat(this.right_trigger).array();
            TODO: 38 bytes until button int. (Not a todo)
          1.0  int var4 = (var2 << 1) + (this.left_stick_button?1:0); -> They're storing all the 0/1 buttons in a single int (4 bytes = 32 bits).
          1.1  var4 = (var4 << 1) + (this.right_stick_button?1:0);
          1.2  var4 = (var4 << 1) + (this.dpad_up?1:0);
          1.3  var4 = (var4 << 1) + (this.dpad_down?1:0);
          1.4  var4 = (var4 << 1) + (this.dpad_left?1:0);
          1.5  var4 = (var4 << 1) + (this.dpad_right?1:0);
          1.6  var4 = (var4 << 1) + (this.a?1:0);
          1.7  var4 = (var4 << 1) + (this.b?1:0);
          2.0  var4 = (var4 << 1) + (this.x?1:0);
          2.1  var4 = (var4 << 1) + (this.y?1:0);
          2.2  var4 = (var4 << 1) + (this.guide?1:0);
          2.3  var4 = (var4 << 1) + (this.start?1:0);
          2.4  var4 = (var4 << 1) + (this.back?1:0);
          2.5  var4 = (var4 << 1) + (this.left_bumper?1:0);
          2.6  var4 = (var4 << 1) + (this.right_bumper?1:0); not sure why they allocate so much.
          4  var1.putInt(var4);
            var1.put(this.user);
     */
}