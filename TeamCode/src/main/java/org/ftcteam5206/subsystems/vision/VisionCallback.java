package org.ftcteam5206.subsystems.vision;

/**
 * Created by tarunsingh on 1/28/17.
 */

public class VisionCallback {
    public boolean hasFinished = false;
    public boolean redIsRight;
    public double blueCenterX;
    public double redCenterX;

    public VisionCallback(){};

    public void update(double blueCenterX, double redCenterX) {
        this.hasFinished = true;
        this.blueCenterX = blueCenterX;
        this.redCenterX = redCenterX;
        this.redIsRight = blueCenterX < redCenterX;
    }
}
