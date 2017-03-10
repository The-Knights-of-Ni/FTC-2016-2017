package org.ftcteam5206.subsystems.vision;

/**
 * Created by tarunsingh on 1/28/17.
 */

public class VisionCallback {
    public boolean hasFinished = false;
    public boolean redIsRight;
    public double redCenterX;
    public double blueCenterX;

    public double centerX;
    public double centerY;

    public boolean beaconIsRed;

    public double[][] ballLocationsRed;
    public double[][] ballLocationsBlue;

    public VisionCallback(){};

    public void update(double [][] ballLocations) {
        this.hasFinished = true;
        this.ballLocationsRed = ballLocations;
    }

    public void update(double redCenterX, double blueCenterX) {
        this.hasFinished = true;
        this.redCenterX = redCenterX;
        this.blueCenterX = blueCenterX;
        this.redIsRight = blueCenterX < redCenterX;
    }

    public void update(boolean beaconIsRed) {
        this.hasFinished = true;
        this.beaconIsRed = beaconIsRed;
    }

    public void updateVortex (double centerX, double centerY) {
        this.centerX = centerX;
        this.centerY = centerY;
        this.hasFinished = true;
    }
}
