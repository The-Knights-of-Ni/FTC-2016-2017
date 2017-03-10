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
import org.ftcteam5206.subsystems.RobotConstants;
import org.ftcteam5206.subsystems.Transport;
import org.ftcteam5206.subsystems.vision.VisionSystem;
import org.ftcteam5206.utils.Maths;

import static org.ftcteam5206.subsystems.Launcher.SemiAutoState.FIRING;
import static org.ftcteam5206.subsystems.Launcher.SemiAutoState.RELOADING;
import static org.ftcteam5206.subsystems.Launcher.SemiAutoState.SPINNING_UP;
import static org.ftcteam5206.subsystems.Launcher.SemiAutoState.TRANSPORTING;

@Autonomous(name = "Mk2Auto", group = "Autonomous")
public class Mk2Auto extends LinearOpMode {
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    public void waitRobot(double seconds){
        double startTime = runtime.seconds();
        while(opModeIsActive() && runtime.seconds()-startTime < seconds){}
    }

    double lastEncoderChangeTime, lastLeftEncoder, lastRightEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.launcher2, robot.voltageSensor, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, robot.transportServo,runtime);
        //BeaconPusher beaconPusher = new BeaconPusher(robot.beaconPusher, runtime);
        VisionSystem visionSystem = new VisionSystem(this);

        int allianceColorInt = 1;
        if(FtcRobotControllerActivity.allianceColor == FtcRobotControllerActivity.AllianceColor.BLUE)
            allianceColorInt = -1;
        telemetry.addData("Robot Yaw", drive.getRobotYaw());
        telemetry.update();
        robot.beaconPusher.setPosition(0);
        transport.setServoPosition(0.3);
        waitForStart();
        robot.resetEncoders();
        runtime.reset();

        lastEncoderChangeTime = runtime.seconds();
        lastLeftEncoder = robot.leftDrive.getCurrentPosition();
        lastRightEncoder = robot.rightDrive.getCurrentPosition();
/*
        double distanceTarget = Math.PI/2.0 * RobotConstants.driveBaseRadius;
        Log.d("autodrive", "Target = " + distanceTarget);
        while(distanceTarget > drive.getAvgDriveEncoderTicks()*RobotConstants.driveTickToDist)
        {
            Log.d("autodrive", "Turning " + drive.getLeftDriveEncoderTicks()*RobotConstants.driveTickToDist + ", " + drive.getRightDriveEncoderTicks()*RobotConstants.driveTickToDist);
            drive.leftDrive.setPower(-0.3);
            drive.rightDrive.setPower(0.3);
        }
        drive.stop();
        //waitRobot(5);
        //Log.d("autodrive", " turn " + drive.getLeftDriveEncoderTicks()*RobotConstants.driveTickToDist);
        while(opModeIsActive()) {
            Log.d("autodrive", "After Turn " + drive.getLeftDriveEncoderTicks()*RobotConstants.driveTickToDist + ", " + drive.getRightDriveEncoderTicks()*RobotConstants.driveTickToDist);
            telemetry.addData("IMU", drive.getRobotYaw());
            telemetry.update();
        }
        */
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
        /*
        while (opModeIsActive() && runtime.seconds() < 10) {
            robot.leftDrive.setPower(1);
            robot.rightDrive.setPower(1);
        }
        drive.stop();
        Log.d("autotest", robot.leftDrive.getCurrentPosition() + " " + robot.rightDrive.getCurrentPosition());
        */
        /*
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setTargetPosition((int) (Maths.degreeToRadians(45)*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist));
        robot.rightDrive.setTargetPosition((int) -(Maths.degreeToRadians(45)*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist));
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        //drive.encoderTurn(90);
        while (opModeIsActive() && (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {
            Log.d("autotest", robot.leftDrive.getCurrentPosition() + " " + robot.rightDrive.getCurrentPosition());
        }
        drive.stop();

        while (opModeIsActive()) {
            telemetry.addData("Heading", drive.getRobotYaw());
            telemetry.update();
        }
        */

        /*
        while (opModeIsActive()) {
            telemetry.addData("Left Encoder", drive.leftDrive.getCurrentPosition());
            telemetry.addData("Right Encoder", drive.rightDrive.getCurrentPosition());
            telemetry.update();
        }
        */

        double driveDelay = .5;
        double segmentStartTime;
        Log.d("autotest", "Starting first drive segment");
        /*
        drive.driveDist(24, 30);
        while(opModeIsActive() && drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        */
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //First drive forward
        robot.leftDrive.setTargetPosition((int) (24/RobotConstants.driveTickToDist));
        robot.rightDrive.setTargetPosition((int) (24/RobotConstants.driveTickToDist));
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);

        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        Log.d("autotest", "Finished first drive segment");
        waitRobot(driveDelay);

        /*
        //Launch 2 balls
        Log.d("autotest", "Spinning up launcher");
        launcher.launcher.setPower(1);
        waitRobot(3);
        Log.d("autotest", "Launching first ball");
        transport.setServoPosition(0.7);
        waitRobot(0.5);
        Log.d("autotest", "Launched first ball");

        Log.d("autotest", "Reset to transport second ball");
        transport.setServoPosition(0);
        waitRobot(0.5);

        while (opModeIsActive()) {
            transport.on();
            if (robot.transportSensor.getValue() == 1) {
                Log.d("autotest", "Second ball on sensor");
                waitRobot(0.25);
                transport.setServoPosition(0.3);
                transport.off();
                Log.d("autotest", "Moved second ball into ready position");
                break;
            }
        }
        waitRobot(0.5);
        transport.setServoPosition(0.7);
        Log.d("autotest", "Launched second ball");
        waitRobot(0.5);
        Log.d("autotest", "Spun down launcher");
        launcher.launcher.setPower(0);

        /*
        drive.encoderTurn(allianceColorInt == 1 ? 315 - drive.getRobotYaw() : 45 - drive.getRobotYaw());
        while(drive.encoderTurnChecker())
        {
            drive.encoderTurnUpdate();
        }
        */

        //First 45 degree turn
        Log.d("autotest", "Starting first turn");
        int offsetTicks = 0;
        if (allianceColorInt == 1)
            offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 315))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        else
            offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 45))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        //Log.d("autotest", "Ticks to go: " + offsetTicks);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        Log.d("autotest", "Finished first turn");
        waitRobot(driveDelay);

        /*
        drive.driveDist(45, 20);
        while(opModeIsActive() && drive.driveDistChecker()){
            drive.driveDistUpdate();
        }
        */
        //Drive toward beacons
        Log.d("autotest", "Starting second drive segment");
        offsetTicks = (int) (38/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        Log.d("autotest", "Finished second drive segment");
        waitRobot(driveDelay);

        /*
        drive.encoderTurn(allianceColorInt == 1 ? 270 - drive.getRobotYaw() : 90 - drive.getRobotYaw());
        while(opModeIsActive() && drive.encoderTurnChecker()){
            drive.encoderTurnUpdate();
        }
        */

        //Second 45 turn to square up with beacons
        Log.d("autotest", "Starting second turn");
        if (allianceColorInt == 1)
            offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 270))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        else
            offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 90))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        Log.d("autotest", "Finished second turn");
        waitRobot(driveDelay);

        /*
        drive.encoderTurn(allianceColorInt == 1 ? 270 - drive.getRobotYaw() : 90 - drive.getRobotYaw());
        while(opModeIsActive() && drive.encoderTurnChecker()){
            drive.encoderTurnUpdate();
        }
        drive.stop();
        waitRobot(driveDelay);
        */


        visionSystem.detectBeacon();
        Log.d("VisionHelper", "Called visionsystem.detectbeacon() in auto");
        while (opModeIsActive() && !visionSystem.visionCallback.hasFinished) {}
        if ((visionSystem.visionCallback.redIsRight && allianceColorInt == 1) || (!visionSystem.visionCallback.redIsRight && allianceColorInt == -1)) //Right beacon position
            robot.beaconPusher.setPosition(0);
        else //Left beacon position
            robot.beaconPusher.setPosition(1);
        Log.d("autotest", "Moved beacon pusher to correct color");


        /*
        if (robot.beaconPusher.getPosition() == 0)
            offsetTicks = (int) (Maths.degreeToRadians(-5)*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        else
            offsetTicks = (int) (Maths.degreeToRadians(5)*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        */

        //Press first beacon
        offsetTicks = (int) (30/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        //Back off first beacon
        offsetTicks = (int) (-15/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        //Check first beacon color
        visionSystem.checkBeacon();
        while (opModeIsActive() && !visionSystem.visionCallback.hasFinished) {}
        if ((visionSystem.visionCallback.beaconIsRed && allianceColorInt == -1) || (!visionSystem.visionCallback.beaconIsRed && allianceColorInt == 1)) { //Incorrect beacon color
            waitRobot(5);

            //Press first beacon
            offsetTicks = (int) (30/RobotConstants.driveTickToDist);
            robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(0.2);
            segmentStartTime = runtime.seconds();
            while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
                if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                    break;
            }
            drive.stop();
            waitRobot(driveDelay);

            //Back off first beacon
            offsetTicks = (int) (-30/RobotConstants.driveTickToDist);
            robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(0.2);
            segmentStartTime = runtime.seconds();
            while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
                if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                    break;
            }
            drive.stop();
            waitRobot(driveDelay);
        }

        //Turn 90 for drive toward second beacon
        offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 0))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        visionSystem.detectBeacon();
        Log.d("VisionHelper", "Called visionsystem.detectbeacon() in auto");
        while (opModeIsActive() && !visionSystem.visionCallback.hasFinished) {}
        if ((visionSystem.visionCallback.redIsRight && allianceColorInt == 1) || (!visionSystem.visionCallback.redIsRight && allianceColorInt == -1)) //Right beacon position
            robot.beaconPusher.setPosition(0);
        else //Left beacon position
            robot.beaconPusher.setPosition(1);
        Log.d("autotest", "Moved beacon pusher to correct color");

        //Drive to second beacon
        offsetTicks = (int) (44/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        //Turn toward second beacon
        if (allianceColorInt == 1)
            offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 270))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        else
            offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 90))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        //offsetTicks = (int) (Maths.degreeToRadians(Maths.smallestSignedAngle(drive.getRobotYaw(), 270))*RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        //Drive and Press second beacon
        offsetTicks = (int) (30/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        //Back off Second beacon
        offsetTicks = (int) (-15/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        segmentStartTime = runtime.seconds();
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
            if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                break;
        }
        drive.stop();
        waitRobot(driveDelay);

        //Check Second beacon color
        visionSystem.checkBeacon();
        while (opModeIsActive() && !visionSystem.visionCallback.hasFinished) {}
        if ((visionSystem.visionCallback.beaconIsRed && allianceColorInt == -1) || (!visionSystem.visionCallback.beaconIsRed && allianceColorInt == 1)) { //Incorrect beacon color
            waitRobot(5);

            //Repress second beacon
            offsetTicks = (int) (30/RobotConstants.driveTickToDist);
            robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(0.2);
            segmentStartTime = runtime.seconds();
            while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
                if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                    break;
            }
            drive.stop();
            waitRobot(driveDelay);

            //Back off second beacon
            offsetTicks = (int) (-30/RobotConstants.driveTickToDist);
            robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(0.2);
            segmentStartTime = runtime.seconds();
            while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {
                if(runtime.seconds() - segmentStartTime > 0.5 && driveIsStalling())
                    break;
            }
            drive.stop();
            waitRobot(driveDelay);
        }

        while(opModeIsActive()) {
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.update();
        }
        /*

        //Spin up launcher
        robot.launcher.setPower(1);
        double currentTime = runtime.seconds();
        while(opModeIsActive() && (runtime.seconds() - currentTime < 1.2)){}

        //wait for 2 seconds to launch balls
//        transport.on();
//        currentTime = runtime.seconds();
//        while(opModeIsActive() && (runtime.seconds() - currentTime < 3.5)){}
//        transport.off();
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
        //Back into cap balll and park (or get ready to get free particle)
        drive.plannedTurn(-45);
        while(drive.plannedTurnChecker()){
            drive.plannedTurnUpdate();
        }
        drive.driveDist(-100);
        while(drive.driveDistChecker()){
            drive.driveDistUpdate();
        }*/
    }

    private boolean driveIsStalling () {
        double leftEncoder = robot.leftDrive.getCurrentPosition();
        double rightEncoder = robot.rightDrive.getCurrentPosition();
        if (leftEncoder != lastLeftEncoder) {
            lastLeftEncoder = leftEncoder;
            lastEncoderChangeTime = runtime.seconds();
        }
        if (rightEncoder != lastRightEncoder) {
            lastRightEncoder = rightEncoder;
            lastEncoderChangeTime = runtime.seconds();
        }
        return runtime.seconds() - lastEncoderChangeTime > 0.5;
    }
}
