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
import org.ftcteam5206.subsystems.vision.VisionSystem;

@Autonomous(name = "Mk2Auto", group = "Autonomous")
public class Mk2Auto extends LinearOpMode {
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.turret, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, runtime);
        BeaconPusher beaconPusher = new BeaconPusher(robot.beaconPusher, runtime);
        VisionSystem visionSystem = new VisionSystem(this);

        waitForStart();
        runtime.reset();

        //Drive Forward (21 in)
        drive.driveDist(27);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        //Launch
        robot.launcher.setPower(1);
        transport.On();
        //wait for 2 seconds to launch balls
        double currentTime = runtime.seconds();
        while(runtime.seconds() - currentTime < 2){}
        transport.Off();
        //Drive forward half a tile after launching
        drive.driveDist(12);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        //Turn 90 degrees (depends on color)
        drive.plannedTurn(-90);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        //Drive to beacon (38 in)
        drive.driveDist(38);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        /*
        //Get Beacon Color
        visionSystem.detectBeacon();
        while(!visionSystem.visionCallback.hasFinished){}
        //Move servo according to beacon orientation and alliance color
        robot.beaconPusher.setPosition(visionSystem.visionCallback.redIsRight ? 1 : 0);
        //Press beacon
        drive.driveDist(4);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        drive.driveDist(-4);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        //Turn 90 degrees
        drive.plannedTurn(90);
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
        }*/
    }
}
