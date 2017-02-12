package org.ftcteam5206.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
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
        Launcher launcher = new Launcher(robot.launcher, robot.turret, robot.turretPot, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, robot.transportServo,runtime);
        //BeaconPusher beaconPusher = new BeaconPusher(robot.beaconPusher, runtime);
        VisionSystem visionSystem = new VisionSystem(this);

        int allianceColorInt = 1;
        if(FtcRobotControllerActivity.allianceColor == FtcRobotControllerActivity.AllianceColor.BLUE)
            allianceColorInt = - 1;

        waitForStart();
        runtime.reset();

        //Drive Forward (36 in)
        drive.driveDist(36);
        while(opModeIsActive() && drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        drive.stop();
        //Spin up launcher
        robot.launcher.setPower(1);
        double currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1.2)){}
        //wait for 2 seconds to launch balls
        transport.on();
        currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 3.5)){}
        transport.off();
        /*
        //Drive forward a tile after launching
        drive.driveDist(25);
        while(opModeIsActive() && drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        drive.stop();
        currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1)){}
        //Turn 45 degrees to square up with first beacon (depends on color)
        drive.plannedTurn(-40*allianceColorInt);
        while(opModeIsActive() && drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        drive.stop();
        currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1)){}
        //Get Beacon color
        visionSystem.detectBeacon();
        while(!visionSystem.visionCallback.hasFinished){}
        //Move servo according to beacon orientation and alliance color
        if((allianceColorInt == 1 && visionSystem.visionCallback.redIsRight) || (allianceColorInt == -1) && !visionSystem.visionCallback.redIsRight) //our alliance color is on right
            robot.beaconPusher.setPosition(1);
        else
            robot.beaconPusher.setPosition(0);
        //Press Beacon
        drive.driveDist(7, 10);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        drive.stop();
        /*
        //Get Beacon Color
        visionSystem.detectBeacon();
        while(!visionSystem.visionCallback.hasFinished){}
        //Move servo according to beacon orientation and alliance color
        robot.beaconPusher.setPosition(visionSystem.visionCallback.redIsRight ? 1 : 0);
        /*
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
