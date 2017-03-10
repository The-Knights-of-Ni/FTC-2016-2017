package org.ftcteam5206.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Drive;
import org.ftcteam5206.subsystems.Intake;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.RobotConstants;
import org.ftcteam5206.subsystems.Transport;

/**
 * Created by tarunsingh on 2/19/17.
 */

@Autonomous(name="2 Ball Launch and Park")
public class Launch2AndPark extends LinearOpMode {
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    public void waitRobot(double seconds){
        double startTime = runtime.seconds();
        while(opModeIsActive() && runtime.seconds()-startTime < seconds){}
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.launcher2, robot.voltageSensor, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, robot.transportServo, runtime);

        int allianceColorInt = 1;
        if (FtcRobotControllerActivity.allianceColor == FtcRobotControllerActivity.AllianceColor.BLUE)
            allianceColorInt = -1;

        robot.beaconPusher.setPosition(0);
        transport.setServoPosition(0.3);
        waitForStart();
        robot.resetEncoders();
        runtime.reset();

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //13 second wait
        waitRobot(13);

        //Drive to launching position
        robot.leftDrive.setTargetPosition((int) (40/RobotConstants.driveTickToDist));
        robot.rightDrive.setTargetPosition((int) (40/RobotConstants.driveTickToDist));
        robot.leftDrive.setPower(0.2);
        robot.rightDrive.setPower(0.2);
        while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {}
        drive.stop();
        waitRobot(0.5);

        //Launch 2 balls
        Log.d("autotest", "Spinning up launcher");
        launcher.launcher.setPower(1);
        waitRobot(4);
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
        transport.setServoPosition(0);
        launcher.launcher.setPower(0);
        waitRobot(1);

        if (allianceColorInt == -1) { //Drive into cap ball and park
            int offsetTicks = (int) (40/RobotConstants.driveTickToDist);
            robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(0.2);
            while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {}
            drive.stop();
        } else { //Drive back
            int offsetTicks = (int) (-20/RobotConstants.driveTickToDist);
            robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + offsetTicks);
            robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + offsetTicks);
            robot.leftDrive.setPower(0.2);
            robot.rightDrive.setPower(0.2);
            while (opModeIsActive() && (robot.leftDrive.isBusy() || robot.rightDrive.isBusy())) {}
            drive.stop();
        }


        /*
        double secondStartTime = runtime.seconds();
        while (opModeIsActive()) {
            transport.on();
            if (runtime.seconds() - secondStartTime > 3)
                break;
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

        //Knock cap ball and park on center vortex
        double secondDistance = 40;
        drive.driveDist(secondDistance, 20);
        while (opModeIsActive() && drive.driveDistChecker()) {
            drive.driveDistUpdate();
        }
        drive.stop();
        */
    }
}
