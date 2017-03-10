package org.ftcteam5206.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Drive;
import org.ftcteam5206.subsystems.Intake;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.RobotConstants;
import org.ftcteam5206.subsystems.Transport;
import org.ftcteam5206.utils.Maths;

/**
 * Created by tarunsingh on 3/5/17.
 */

@Autonomous(name="Mk3Auto")
public class Mk3Auto extends LinearOpMode {
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //Init robot subsystems
        robot.init(hardwareMap);
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.launcher2, robot.voltageSensor, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, robot.transportServo,runtime);

        //Init servos
        robot.beaconPusher.setPosition(0);
        transport.setServoPosition(0.3);

        waitForStart();
        robot.resetEncoders();
        runtime.reset();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int offsetTicks = (int) (Maths.degreeToRadians(-90)* RobotConstants.driveBaseRadius/RobotConstants.driveTickToDist);
        robot.leftDrive.setTargetPosition(offsetTicks);
        robot.rightDrive.setTargetPosition(-offsetTicks);
        robot.leftDrive.setPower(.3);
        robot.rightDrive.setPower(.3);

        while(opModeIsActive() && (robot.leftDrive.isBusy()||robot.rightDrive.isBusy())) {}
        drive.stop();

        while(opModeIsActive()) {
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.update();
        }
    }

    private void waitRobot(double seconds) {
        double startTime = runtime.seconds();
        while(opModeIsActive() && runtime.seconds()-startTime < seconds){}
    }
}
