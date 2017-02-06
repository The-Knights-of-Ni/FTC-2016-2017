package org.ftcteam5206.subsystems;

/**
 * Created by Dev on 11/25/2016.
 */
import static org.ftcteam5206.utils.Maths.pi;

public final class RobotConstants {
    /*
    All units are in inches/seconds/lbs
     */
    //MOTOR
    public final static int neverRestPPR = 7;
    public final static int neverRestCPR = 4;
    //DRIVE
    public final static int wheelRadius = 2;
    public final static double driveBaseRadius = 8.125;
    public final static double maxDriveVelocity = 67;
    public final static double maxDriveAcceleration = 150;
    public final static double driveDistTolerance = 0.5;
    public final static int driveGearRatio = 20;
    public final static int drivePPR = neverRestCPR*neverRestPPR*driveGearRatio;
    public final static double distPerRev = wheelRadius*2*pi;
    public final static double driveTickToDist = distPerRev/drivePPR;

    //LAUNCHER
    public final static int launcherGearRatio = 2;
    public final static int launcherPPR = neverRestCPR*neverRestPPR*launcherGearRatio;
    public final static int turretRingGear = 455;
    public final static int turretMotorGear = 100;
    public final static int turretPotGear = 40;

}
