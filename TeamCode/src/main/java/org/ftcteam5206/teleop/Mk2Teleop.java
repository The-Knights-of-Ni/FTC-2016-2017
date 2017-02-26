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

import static org.ftcteam5206.subsystems.Launcher.LauncherState.OPEN_LOOP;
import static org.ftcteam5206.subsystems.Launcher.LauncherState.SEMI_AUTO;
import static org.ftcteam5206.subsystems.Launcher.SemiAutoState.*;
//import org.firstinspires.ftc.teamcode.R;

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
        robot.phone.setPosition(0);
        transport.setServoPosition(servoPosition);
        robot.resetEncoders();
        visionSystem.disableCamera();
        waitForStart();
        runtime.reset();

        boolean wheelHasSpunUp = false;

        //drive.zeroIMU();
        //Auto Drive
        boolean driveAutoInitializing = false;
        double afterTurnLogs = Double.MAX_VALUE;
        //Semi Auto Launcher
        Launcher.SemiAutoState semiAutoStatus = TRANSPORTING;
        double ballLaunchStartTime = 0;
        double ballGrabTime = runtime.seconds();
        boolean emptyChamber = false;
        double shotConfirmTime = 0;
        double lastRPM = 0, counter = 0;
        double servoUpdateRate = 0;
        boolean switchHasBeenTriggered = false;

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
                    drive.rightDrive.setPower(Range.clip(-sticks.x - sticks.y, -0.75, 0.75));
                    drive.leftDrive.setPower(Range.clip(sticks.x - sticks.y, -0.75, 0.75));
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
            if(pad2.singlePress(pad2.buttons.DPAD_UP)) {
                launcher.setLauncherState(OPEN_LOOP);
                emptyChamber = false;
            }
            if(pad2.singlePress(pad2.buttons.DPAD_DOWN))
                launcher.setLauncherState(SEMI_AUTO);
            if(pad2.press(pad2.buttons.Y) && launcher.getLauncherState() == SEMI_AUTO){
                semiAutoStatus = EXHAUSTING;
            }
            //Launcher state machine
            switch (launcher.getLauncherState()) {
                case STOPPED:
                    break;
                case OPEN_LOOP: //Each Shot must be confirmed
                    if (pad2.toggle(pad2.buttons.X)) {
                        robot.launcher.setPower(1);
                        telemetry.addData("Launcher", "On");
                    } else {
                        robot.launcher.setPower(0);
                        telemetry.addData("Launcher", "off");
                    }
                    if (pad2.press(pad2.buttons.LEFT_TRIGGER)) {
                        ballLaunchStartTime = runtime.seconds();
                        servoPosition = 0.7;
                        transport.setServoPosition(servoPosition);
                    }
                    if (servoPosition == 0.7) { //Flipper is ready to launch
                        Log.d("autodrive", "Time since button press " + (runtime.seconds() - ballLaunchStartTime));
                        if (runtime.seconds() - ballLaunchStartTime > 0.25) {
                            servoPosition = 0;
                            transport.setServoPosition(servoPosition);
                        }
                    }
                    break;
                case SEMI_AUTO: //After Confirm we spin up, empty the chamber (loop control), and then stop after confirm is pressed again
                    if (pad2.singlePress(pad2.buttons.X)) {
                        emptyChamber = !emptyChamber;
                        shotConfirmTime = runtime.seconds();
                        semiAutoStatus = TRANSPORTING;
                        if(!emptyChamber)
                            servoPosition = 0;
                    }
                    if (emptyChamber) {
                        switch (semiAutoStatus) {
                            case EXHAUSTING:
                                transport.transportMotor.setPower(-0.5);
                                break;
                            case TRANSPORTING:
                                //TODO: Take control from drivers so they don't mess this up.
                                transport.on();
                                //Log.d("autolaunch", "Turning transport on");
                                if (transport.getServoPosition() == 0.3) {
                                    shotConfirmTime = runtime.seconds();
                                    if (wheelHasSpunUp)
                                        semiAutoStatus = FIRING;
                                    else
                                        semiAutoStatus = SPINNING_UP;
                                }
                                break;
                            case SPINNING_UP:
                                transport.off();
                                //launcher.flywheel.spinUpToRPM(3000);//FIXME: This method is broken
                                launcher.launcher.setPower(1);
                                if (runtime.seconds() - shotConfirmTime > 3) {
                                    shotConfirmTime = runtime.seconds();
                                    semiAutoStatus = FIRING;
                                    wheelHasSpunUp = true;
                                }
                                break;
                            case FIRING:
                                servoPosition = 0.7;
                                double deltaTime = runtime.seconds() - shotConfirmTime;
                                if (deltaTime > 1.75 && deltaTime < 2)
                                    intake.intakeReverse();
                                else
                                    transport.off();
                                if (runtime.seconds() - shotConfirmTime > 2)//This should cause a jam to not be a problem, otherwise drivers will manually deal with it.
                                    semiAutoStatus = RELOADING;
                                    break;
                            case RELOADING:
                                transport.off();
                                servoPosition = 0.0;
                                semiAutoStatus = TRANSPORTING;
                                break;
                        }
                    } else {
                        launcher.flywheel.spinDown();
                        wheelHasSpunUp = false;
                    }
                    break;
            }

            // Intake/transport control
            if(!emptyChamber) {
                Log.d("autolaunch", "Manual intake/transport control");
                if (pad2.press(pad2.buttons.Y)) //Reverse intake
                    intake.intakeReverse();
                else if (pad1.toggle(pad1.buttons.A)) //Intake on
                    intake.intakeOn();
                else //Intake off
                    intake.intakeOff();
                if(pad2.press(pad2.buttons.LEFT_TRIGGER)) {
                    ballLaunchStartTime = runtime.seconds();
                    servoPosition = 0.7;
                    transport.setServoPosition(servoPosition);
                }
            }


            //Flipper control
            if(pad2.singlePress(pad2.buttons.RIGHT_TRIGGER)){
                servoPosition = 0;
            }

            if(servoPosition == 0.7)
                servoUpdateRate = runtime.seconds();

            if (servoPosition == 0 && runtime.seconds() - servoUpdateRate > 0.25){ //Flipper is down
                if (!switchHasBeenTriggered && robot.transportSensor.getValue() == 1) { //Ball is on flipper
                    switchHasBeenTriggered = true;
                    ballGrabTime = runtime.seconds();
                }
                if (switchHasBeenTriggered && runtime.seconds() - ballGrabTime > 0.2){
                    ballGrabTime = runtime.seconds();
                    servoPosition = 0.3;
                    switchHasBeenTriggered = false;
                }
            }
//            else if (servoPosition == 0.7){ //Flipper is ready to launch
//                Log.d("autodrive", "Time since button press " + (runtime.seconds()-ballLaunchStartTime));
//                if (runtime.seconds()-ballLaunchStartTime > 0.25) {
//                    servoPosition = 0;
//                }
//            }

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

            //Hood control
            if (pad2.press(pad2.buttons.LEFT_BUMPER))
                robot.hood.setPosition(0);
            else if (pad2.press(pad2.buttons.RIGHT_BUMPER))
                robot.hood.setPosition(1);

            int secondsRemaining = (int) (120-runtime.seconds());
            telemetry.addData("Time", secondsRemaining/60 + ":" + secondsRemaining%60);
            telemetry.addData("Transport Servo", servoPosition);
            telemetry.addData("Ball Sensor", robot.transportSensor.getValue());
            telemetry.addData("Launcher FSM", launcher.getLauncherState());
            if(launcher.getLauncherState() == SEMI_AUTO)
                telemetry.addData("Semi Auto FSM", semiAutoStatus);
            telemetry.addData("Intake FSM", intake.getIntakeState());
            telemetry.addData("Drive FSM", drive.getDriveState());
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.addData("Launcher Ticks", robot.launcher.getCurrentPosition());
            telemetry.update();
            double rpm = launcher.flywheel.getRPM();
            /*
            if (rpm != lastRPM) {
                Log.d("autotest", String.valueOf(rpm));
                lastRPM = rpm;
            }
            */
        }
    }
}
