package org.ftcteam5206.teleop;

/**
 * Created by Dev on 1/20/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Cap;
import org.ftcteam5206.subsystems.Drive;
import org.ftcteam5206.subsystems.Intake;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.Transport;
import org.ftcteam5206.utils.Button.Button;
import org.ftcteam5206.utils.Button.ButtonHandler;
import org.ftcteam5206.utils.Button.Buttons;
import org.ftcteam5206.utils.JoystickSmoother;
import org.ftcteam5206.utils.Maths;
import org.ftcteam5206.utils.vectors.vector2d;
import org.ftcteam5206.utils.vectors.vector3d;

@TeleOp(name="TeleOp", group="TeleOP")
public class Mk2Teleop extends LinearOpMode{
    KingArthur robot = new KingArthur();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException
    {
        ButtonHandler pad1 = new ButtonHandler();
        ButtonHandler pad2 = new ButtonHandler();

        robot.init(hardwareMap);
        //Init Subsystem Controllers
        Drive drive = new Drive(robot.leftDrive, robot.rightDrive, robot.imu, runtime);
        Launcher launcher = new Launcher(robot.launcher, robot.hood, runtime);
        Intake intake = new Intake(robot.intakeTransport, runtime);
        Transport transport = new Transport(robot.intakeTransport, runtime);
        Cap cap = new Cap(robot.capRelease,robot.forkRelease, robot.clasp, runtime);

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
                    drive.rightDrive.setPower(sticks.x - sticks.y);
                    drive.leftDrive.setPower(-sticks.x - sticks.y);
                    break;
                case AUTO:
                    if (driveAutoInitializing){
                        drive.absTurn(180);
                        driveAutoInitializing = false;
                    }
                    if(drive.absTurnChecker(10)){
                        drive.absTurnUpdate();
                    }
                    else{
                        drive.setDriveState(Drive.DriveState.OPEN_LOOP);
                    }


                    break;
            }
            if (pad1.toggle(pad1.buttons.A)){
                drive.setDriveState(Drive.DriveState.AUTO);
                driveAutoInitializing = true;
            }
            else
                drive.setDriveState(Drive.DriveState.OPEN_LOOP);


            orient = new vector3d(robot.imu.getAngularOrientation().firstAngle, robot.imu.getAngularOrientation().secondAngle, robot.imu.getAngularOrientation().thirdAngle);
            telemetry.addData("Angle 1", orient.x);
            telemetry.addData("Robot Yaw", drive.getRobotYaw());
            telemetry.addData("Angle 2", orient.y);
            telemetry.addData("Angle 3", orient.z);
            telemetry.update();
        }
    }
}
