package org.ftcteam5206.utils.Button;

/**
 * Created by Dev on 12/17/2016.
 */

public class Button {
    final int value;
    boolean status;

    Button(int value, boolean status){
        this.value = value;
        this.status = status;
    }

    public void setStatus(boolean status) {
        this.status = status;
    }

    public boolean toggle(){
        this.status = !this.status;
        return status;
    }
}
