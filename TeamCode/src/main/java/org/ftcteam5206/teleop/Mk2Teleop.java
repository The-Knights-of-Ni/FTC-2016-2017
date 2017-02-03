package org.ftcteam5206.teleop;

import android.app.Activity;
import android.util.Log;
import android.view.View;
import android.widget.RadioButton;
import android.widget.SeekBar;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Cap;
import org.ftcteam5206.subsystems.Drive;
import org.ftcteam5206.subsystems.Intake;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.RobotConstants;
import org.ftcteam5206.subsystems.Transport;
import org.ftcteam5206.subsystems.Vision;
import org.ftcteam5206.utils.Button.ButtonHandler;
import org.ftcteam5206.utils.JoystickSmoother;
import org.ftcteam5206.utils.vectors.vector2d;
import org.ftcteam5206.utils.vectors.vector3d;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name="TeleOp", group="TeleOP")
public class Mk2Teleop extends LinearOpMode{
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    double lastRuntime = 0;
    double lastEncoderTicks = 0;
    double encoderRefreshRate = 10;

    public enum AllianceColor {
        RED, BLUE
    }

    private RadioButton allianceColorRed, allianceColorBlue;
    public AllianceColor allianceColor;

    @Override
    public void runOpMode() throws InterruptedException
    {
        ButtonHandler pad1 = new ButtonHandler();
        ButtonHandler pad2 = new ButtonHandler();

        robot.init(hardwareMap);
        //Init Subsystem Controllers
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.turret, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, runtime);
        Cap cap = new Cap(robot.capRelease,robot.forkRelease, robot.clasp, runtime);

        Vision vision = new Vision(this);

        allianceColorRed = (RadioButton) ((Activity) hardwareMap.appContext).findViewById(R.id.allianceColorRed);
        allianceColorBlue = (RadioButton) ((Activity) hardwareMap.appContext).findViewById(R.id.allianceColorBlue);
        allianceColorRed.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                allianceColor = AllianceColor.RED;
            }
        });

        allianceColorBlue.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                allianceColor = AllianceColor.BLUE;
            }
        });

        waitForStart();
        runtime.reset();
        //drive.zeroIMU();

        vector3d orient;
        boolean driveAutoInitializing = true;
        while (opModeIsActive()) {
            //Button Try Catch
            try {
                pad1.updateButtons(gamepad1.toByteArray(), gamepad1.left_trigger, gamepad1.right_trigger);
                pad2.updateButtons(gamepad2.toByteArray(), gamepad2.left_trigger, gamepad2.right_trigger);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            //Drive State Machine
            switch (drive.getDriveState()) {
                case STOPPED:
                    break;
                case OPEN_LOOP:
                    vector2d sticks = JoystickSmoother.smoothJoysticksBezierStyle(new vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y));
                    drive.rightDrive.setPower(-sticks.x - sticks.y);
                    drive.leftDrive.setPower(sticks.x - sticks.y);
                    break;
                case AUTO:
                    if (driveAutoInitializing){
                        vision.detectBeacon();
                        driveAutoInitializing = false;
                    }
                    if(!vision.visionCallback.hasFinished){
                    }
                    else{
                        if((vision.visionCallback.redIsRight && allianceColor == AllianceColor.RED) || (!vision.visionCallback.redIsRight && allianceColor == AllianceColor.BLUE))
                            robot.beaconPusher.setPosition(0);
                        else
                            robot.beaconPusher.setPosition(1);
                        pad1.buttons.Y.setStatus(false);
                    }
                    break;
            }
            if(pad1.singlePress(pad1.buttons.Y)){
                driveAutoInitializing = true;
            }
            if (pad1.toggle(pad1.buttons.Y)){
                drive.setDriveState(Drive.DriveState.AUTO);
            }
            else
                drive.setDriveState(Drive.DriveState.OPEN_LOOP);

            //Launcher control
            if(pad1.toggle(pad1.buttons.X)) {
                robot.launcher.setPower(1.0);
                telemetry.addData("Launcher", "On");
            }
            else {
                robot.launcher.setPower(0);
                telemetry.addData("Launcher", "Off");
            }

            if(runtime.seconds() - lastRuntime > (1/encoderRefreshRate)) {
                double encoderTicksSinceLast = robot.launcher.getCurrentPosition() - lastEncoderTicks;
                double timeSinceLast = runtime.seconds() - lastRuntime;
                double revolutions = encoderTicksSinceLast / RobotConstants.launcherPPR;
                double minutes = timeSinceLast / 60;
                double RPM = revolutions/minutes;
                lastRuntime = runtime.seconds();
                lastEncoderTicks = robot.launcher.getCurrentPosition();
            }

            orient = new vector3d(robot.imu.getAngularOrientation().firstAngle, robot.imu.getAngularOrientation().secondAngle, robot.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("Drive State Machine", drive.getDriveState());
            telemetry.addData("Angle 1", orient.x);
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.addData("Turret Pot", robot.turretPot.getVoltage());
            telemetry.addData("Angle 3", orient.z);
            telemetry.update();
        }
    }

    private void onRadioButtonClicked() {

    }
}
