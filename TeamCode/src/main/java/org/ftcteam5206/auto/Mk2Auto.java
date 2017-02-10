package org.ftcteam5206.auto;

import android.util.Log;

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
import org.ftcteam5206.utils.Maths;

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
        Transport transport = new Transport(robot.intakeTransport, runtime);
        //BeaconPusher beaconPusher = new BeaconPusher(robot.beaconPusher, runtime);
        VisionSystem visionSystem = new VisionSystem(this);

        int allianceColorInt = 1;
        if(FtcRobotControllerActivity.allianceColor == FtcRobotControllerActivity.AllianceColor.BLUE)
            allianceColorInt = - 1;
        telemetry.addData("Robot Yaw", drive.getRobotYaw());
        telemetry.update();
        waitForStart();
        runtime.reset();
        //double robotYaw = -robot.imu.getAngularOrientation().firstAngle;
        /*double targetAngle = -robot.imu.getAngularOrientation().firstAngle + 90;
        double turnStartTime = runtime.seconds();
        Log.d("autodrive", "Planned Turn Starting. Target is " + targetAngle + " we're currently at " + -robot.imu.getAngularOrientation().firstAngle);
        robot.dim.setLED(1, false);
        while(Math.abs(Maths.smallestSignedAngle(-robot.imu.getAngularOrientation().firstAngle, targetAngle)) > 1.0){
            Log.d("autodrive", (runtime.seconds() - turnStartTime) + ", " +  -robot.imu.getAngularOrientation().firstAngle + ", "  + Maths.smallestSignedAngle(-robot.imu.getAngularOrientation().firstAngle, targetAngle));
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(-0.2);
        }
        robot.dim.setLED(1, true);
        robot.rightDrive.setPower(0.0);
        robot.leftDrive.setPower(0.0);
        Log.d("autodrive", "Turn finished");
        double currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1)){
            Log.d("autodrive", (runtime.seconds() - turnStartTime) + ", " +  -robot.imu.getAngularOrientation().firstAngle + ", "  + Maths.smallestSignedAngle(-robot.imu.getAngularOrientation().firstAngle, targetAngle));
        }
        Log.d("autodrive", "angle 1s after turn " + -robot.imu.getAngularOrientation().firstAngle);
        Log.d("autodrive", "power 1s after turn " + robot.rightDrive.getPower());
        //Drive Forward (36 in)*/

        drive.driveDist(20, 20);
        while(opModeIsActive() && drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        drive.stop();
        double currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1)){}
        drive.plannedTurn(90);
        while(opModeIsActive() && drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
            sleep(5);
        }
        drive.stop();
        currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1)){}
        telemetry.addData("Robot Yaw", drive.getRobotYaw());
        telemetry.update();
        drive.stop();
        drive.plannedTurn(90 - drive.getRobotYaw());
        while(opModeIsActive() && Maths.smallestSignedAngle(drive.getRobotYaw(),drive.targetAngle) < 0){
            drive.plannedTurnUpdate();
        }
        drive.stop();
        currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1)){}
        telemetry.addData("Robot Yaw", drive.getRobotYaw());
        telemetry.update();
        while(opModeIsActive());
//        drive.driveDist(40, 20);
//        while(opModeIsActive() && drive.driveDistChecker()){
//            drive.driveDistUpdate();
//        }
//        drive.stop();
        /*drive.driveDist(36);
        while(opModeIsActive() && drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        drive.stop();

        //Spin up launcher
        robot.launcher.setPower(1);
        double currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1.2)){}
        //wait for 2 seconds to launch balls
        transport.On();
        currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 3.5)){}
        transport.Off();*/

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
