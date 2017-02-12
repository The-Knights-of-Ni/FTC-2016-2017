package org.ftcteam5206.teleop;

import android.app.Activity;
import android.util.Log;
import android.view.View;
import android.widget.RadioButton;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Cap;
import org.ftcteam5206.subsystems.Drive;
import org.ftcteam5206.subsystems.Intake;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.PlannedPath;
import org.ftcteam5206.subsystems.RobotConstants;
import org.ftcteam5206.subsystems.Transport;
import org.ftcteam5206.subsystems.vision.VisionSystem;
import org.ftcteam5206.utils.Button.ButtonHandler;
import org.ftcteam5206.utils.JoystickSmoother;
import org.ftcteam5206.utils.vectors.vector2d;
import org.ftcteam5206.utils.vectors.vector3d;

import org.firstinspires.ftc.teamcode.R;

@TeleOp(name="TeleOp", group="TeleOP")
public class Mk2Teleop extends LinearOpMode{
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        ButtonHandler pad1 = new ButtonHandler();
        ButtonHandler pad2 = new ButtonHandler();

        robot.init(hardwareMap);
        //Init Subsystem Controllers
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.turret, robot.turretPot, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, robot.transportServo, runtime);
        double servoPosition = 0;
        robot.beaconPusher.setPosition(0.5);
        transport.setServoPosition(servoPosition);
        robot.resetEncoders();
        waitForStart();
        runtime.reset();
        //drive.zeroIMU();
        boolean driveAutoInitializing = false;
        double afterTurnLogs = Double.MAX_VALUE;
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
                    if(driveAutoInitializing){
                        //drive.plannedTurn(90);
                        drive.driveDist(24, 20);
                        driveAutoInitializing = false;
                    }
                    if(drive.driveDistChecker()){
                        drive.driveDistUpdate();
                    } else {
                        Log.d("autodrive", "Turn finished");
                        pad1.buttons.Y.setStatus(false);
                        afterTurnLogs = runtime.seconds();
                    }
                    break;
            }
            if(runtime.seconds() - afterTurnLogs > 5 ){
                afterTurnLogs = Double.MAX_VALUE;
                Log.d("autodrive", "angle 5s after turn " + drive.getRobotYaw());
                Log.d("autodrive", "power 5s after turn " + drive.rightDrive.getPower());
            }


            if(pad1.toggle(pad1.buttons.Y)){
                drive.setDriveState(Drive.DriveState.AUTO);
            } else
                drive.setDriveState(Drive.DriveState.OPEN_LOOP);
            if(pad1.singlePress(pad1.buttons.Y)){
                driveAutoInitializing = true;
            }

            /*
            //Intake State Machine
            switch (intake.getIntakeState()) {
                case STOPPED:
                    break;
                case OPEN_LOOP:
                    //Enter launch mode
                    /*
                    if(pad2.toggle(pad2.buttons.X)) { //Enter launch mode
                        intake.setIntakeState(Intake.IntakeState.AUTO);
                        pad2.buttons.X.setStatus(false);
                    } else if (pad2.press(pad2.buttons.Y))
                        intake.intakeReverse();
                    else if (pad1.toggle(pad1.buttons.A)) {
                        intake.intakeOn();
                    }
                    else
                        intake.intakeOff();
                    break;

                    if (pad2.press(pad2.buttons.Y))
                        intake.intakeReverse();
                    else if (pad1.press(pad1.buttons.A)) {
                        intake.intakeOn();
                        telemetry.addData("Intake", "On");
                    } else {
                        intake.intakeOff();
                        telemetry.addData("Intake", "Off");
                    }
                    break;
                case AUTO:
                    intake.intakeOff();
                    if(pad2.toggle(pad2.buttons.A)) //Confirm launch
                        intake.intakeOn();
                    if(pad2.toggle(pad2.buttons.X)) { //Exit launch mode
                        intake.setIntakeState(Intake.IntakeState.OPEN_LOOP);
                    }
                    break;
            }
            */

            //Launcher state machine
            switch (launcher.getLauncherState()) {
                case STOPPED:
                    break;
                case OPEN_LOOP:
                    if (pad2.toggle(pad2.buttons.X)) {
                        robot.launcher.setPower(1);
                        telemetry.addData("Launcher", "On");
                    } else {
                        robot.launcher.setPower(0);
                        telemetry.addData("Launcher", "off");
                    }
                    break;
            }

            //Turret control
            robot.turret.setPower(.3 * gamepad2.left_stick_x);

            //Transport servo control
            /*
            double newPosition = transport.getServoPosition() -.1*gamepad1.right_stick_y;
            Range.clip(newPosition, 0, 1);
            transport.setServoPosition(newPosition);
            */

            servoPosition += -.01*gamepad1.right_stick_y;
            servoPosition = Range.clip(servoPosition, 0, 1);
            robot.transportServo.setPosition(servoPosition);

            // Intake/transport control
            if (pad2.press(pad2.buttons.Y)) //Reverse intake
                intake.intakeReverse();
            else if (pad1.press(pad1.buttons.A)) //Intake on
                intake.intakeOn();
            else //Intake off
                intake.intakeOff();

            //Flipper control
            if (servoPosition == 0) //Flipper is down
                if (robot.transportSensor.getValue() == 1) { //Ball is on flipper
                    servoPosition = 0.3;
                    transport.setServoPosition(servoPosition);
                }
            else if (servoPosition == 0.7) //Flipper is ready to launch
                if (robot.transportSensor.getValue() == 0) {
                    servoPosition = 0;
                    transport.setServoPosition(servoPosition);
                }

            if(pad2.press(pad2.buttons.LEFT_TRIGGER)) {
                servoPosition = 0.7;
                transport.setServoPosition(servoPosition);
            }

            telemetry.addData("Transport Servo", servoPosition);
            telemetry.addData("Ball Sensor", robot.transportSensor.getValue());
            telemetry.addData("Intake State Machine", intake.getIntakeState());
            telemetry.addData("Drive State Machine", drive.getDriveState());
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.addData("Turret Angle", launcher.turret.getAngle());
            telemetry.update();
        }
    }
}
