package org.ftcteam5206.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.BeaconPusher;
import org.ftcteam5206.subsystems.Drive;
import org.ftcteam5206.subsystems.Intake;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.Transport;

@Autonomous(name = "Mk2Auto", group = "Autonomous")
public class Mk2Auto extends LinearOpMode {
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, runtime);
        BeaconPusher beaconPusher = new BeaconPusher(robot.beaconPusher, runtime);

        waitForStart();
        runtime.reset();

        //Drive Forward (30 in)
        drive.driveDist(30);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        //Launch
        launcher.flywheel.spinUpToRPM(1600);//TODO: Replace with lookup
        transport.On();
        transport.Off();
        //Turn 45 degrees (depends on color)
        drive.plannedTurn(45);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        //Drive to goal (60 in)
        drive.driveDist(60);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        //Turn 45 degrees (maybe swing turn)
        drive.plannedTurn(45);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        //Get Beacon Color
        //Press beacon
        //Turn 90 degrees
        drive.plannedTurn(-90);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        //Drive to next beacon (TODO close with line sensor)
        drive.driveDist(40);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        //Turn 90 degrees
        drive.plannedTurn(90);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        //Get Beacon Color
        //Press beacon
        //Back into cap ball and park (or get ready to get free particle)
        drive.plannedTurn(-45);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        drive.driveDist(-100);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
    }
}
