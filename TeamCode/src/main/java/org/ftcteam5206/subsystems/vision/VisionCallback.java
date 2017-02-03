package org.ftcteam5206.subsystems.vision;

/**
 * Created by tarunsingh on 1/28/17.
 */

public class VisionCallback {
    public boolean hasFinished = false;
    public boolean redIsRight;
    public double redCenterX;
    public double blueCenterX;

    public VisionCallback(){};

    public void update(double redCenterX, double blueCenterX) {
        this.hasFinished = true;
        this.redCenterX = redCenterX;
        this.blueCenterX = blueCenterX;
        this.redIsRight = blueCenterX < redCenterX;
    }
}
