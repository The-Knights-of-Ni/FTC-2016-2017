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
        Launcher launcher = new Launcher(robot.launcher, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, robot.transportServo, runtime);
        Cap cap = new Cap(robot.forkReleaseLeft, robot.forkReleaseRight, robot.clasp, robot.capMotor, runtime);
        VisionSystem visionSystem = new VisionSystem(this);

        double servoPosition = 0;
        robot.beaconPusher.setPosition(0.5);
        transport.setServoPosition(servoPosition);
        robot.resetEncoders();
        waitForStart();
        runtime.reset();
        //drive.zeroIMU();

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
                    break;
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

            //Transport servo control
            /*
            double newPosition = transport.getServoPosition() -.1*gamepad1.right_stick_y;
            Range.clip(newPosition, 0, 1);
            transport.setServoPosition(newPosition);
            */
            /*
            servoPosition += -.01*gamepad1.right_stick_y;
            servoPosition = Range.clip(servoPosition, 0, 1);
            robot.transportServo.setPosition(servoPosition);
            */

            // Intake/transport control
            if (pad2.press(pad2.buttons.Y)) //Reverse intake
                intake.intakeReverse();
            else if (pad1.press(pad1.buttons.A)) //Intake on
                intake.intakeOn();
            else //Intake off
                intake.intakeOff();

            //Flipper control
            if (servoPosition == 0) { //Servo is down
                if (robot.transportSensor.getValue() == 1) { //Ball is on switch
                    servoPosition = 0.3;
                }
            } else if (servoPosition == 0.3) {
                if (pad2.press(pad2.buttons.LEFT_TRIGGER)) { //Driver shoots ball
                    servoPosition = 0.7;
                }
            } else if (servoPosition == 0.7) {
                if (robot.transportSensor.getValue() == 0) { //Ball has been launched
                    servoPosition = 0;
                }
            }

            transport.setServoPosition(servoPosition);

            //Cap control
            switch(cap.capState) {
                case STOPPED:
                    if (pad1.toggle(pad1.buttons.Y)) { //Enter cap mode and release forks
                        cap.capState = Cap.CapState.FORKS_DOWN;
                        cap.releaseForks();
                        pad1.buttons.Y.setStatus(false);
                    }
                    break;
                case FORKS_DOWN:
                    if (pad1.toggle(pad1.buttons.Y)) { //Deploy clasp
                        cap.capState = Cap.CapState.CLASP_ON;
                        cap.deployClasp();
                        pad1.buttons.Y.setStatus(false);
                    }
                    break;
                case CLASP_ON:
                    if (pad1.toggle(pad1.buttons.Y)) { //Lift cap ball
                        cap.capState = Cap.CapState.LIFTING;
                        cap.liftCap();
                        pad1.buttons.Y.setStatus(false);
                    }
                    break;
                case LIFTING:
                    break;
                case LIFTED:
                    break;
            }

            telemetry.addData("Transport Servo", servoPosition);
            telemetry.addData("Ball Sensor", robot.transportSensor.getValue());
            telemetry.addData("Intake State Machine", intake.getIntakeState());
            telemetry.addData("Drive State Machine", drive.getDriveState());
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.update();
        }
    }
}
