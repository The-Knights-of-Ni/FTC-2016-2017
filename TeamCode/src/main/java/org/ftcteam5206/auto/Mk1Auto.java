package org.ftcteam5206.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.RobotName;
import org.ftcteam5206.subsystems.*;
import org.ftcteam5206.utils.*;
import org.ftcteam5206.utils.vectors.*;

/**
 * Mk 1 Autonomous
 * Get both beacons + score 2 balls in 10s.
 */
@Autonomous(name="Mk1 Autonomous")
public class Mk1Auto extends LinearOpMode{
    RobotName robot = new RobotName();
    ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        robot.resetEncoders();

        telemetry.addData("Status", "IMU Calibrating");
        telemetry.update();
        robot.imuCalibrate();

        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.hood, runtime);
        Intake intake = new Intake(robot.intake, runtime);
        Transport transport = new Transport(robot.leftTransport, robot.rightTransport, robot.transportIn, robot.transportOut, runtime);
        BeaconPusher beaconPusher = new BeaconPusher(robot.beaconPusher, runtime);
        Cap cap = new Cap(robot.capRelease, robot.forkRelease, robot.clasp, runtime);

        waitForStart();
        runtime.reset();
        //Spin up and Launch
        //If we want to intake partners ball do so.
        //Drive to the first beacon. Should replace this with a function, but loop allows for other things to happen simultaneously
        PlannedPath pathToFirstBeacon = new PlannedPath(80);
        vector2d drivePWM = new vector2d(0,0);
        double startTime = runtime.time();
        double startDist = robot.getLeftDriveDist();//TODO: Replace with 2D vector
        do{
            drivePWM = pathToFirstBeacon.getPWM(robot.getLeftDriveDist() - startDist, runtime.time()-startTime, 0);//TODO: Add heading
            robot.setLeftDrivePWM(drivePWM.x);
            robot.setRightDrivePWM(drivePWM.y);
        }while(drivePWM.x != 0 && drivePWM.y != 0);//TODO: Need out of time case on loop
        //Turn and face the beacon
        //Use vision to set red/blue
        //Move servo to right position
        //Drive forward slightly (1DFF?) (Maybe confirm that we flipped a color)
        //Turn 91
        //Drive to next beacon
        //Turn back
        //Use vision
        //Move servo
        //Drive forward slightly
        //Turn and back into cap ball
    }
}
