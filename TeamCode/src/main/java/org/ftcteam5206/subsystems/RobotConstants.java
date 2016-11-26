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
    //DRIVE
    public final static int wheelRadius = 2;
    public final static double maxDriveVelocity = 18;
    public final static double maxDriveAcceleration = 10;
    public final static double driveDistTolerance = 0.5;
    public final static int driveGearRatio = 20;
    public final static int drivePPR = neverRestPPR*driveGearRatio;
    public final static double distPerRev = wheelRadius*2*pi;
    public final static double driveTicksToDist = drivePPR*distPerRev;

}
