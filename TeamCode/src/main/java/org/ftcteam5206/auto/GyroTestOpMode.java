package org.ftcteam5206.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Drive;

/**
 * Created by tarunsingh on 2/4/17.
 */

@TeleOp(name="Gyro Test")
public class GyroTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        KingArthur robot = new KingArthur();
        //ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive() && -robot.imu.getAngularOrientation().firstAngle < 90) {
            robot.leftDrive.setPower(.2);
            robot.rightDrive.setPower(-.2);
            telemetry.addData("Heading", -robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        while(opModeIsActive()) {
            telemetry.addData("Heading", -robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }

    }
}
