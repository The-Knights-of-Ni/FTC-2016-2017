package org.ftcteam5206.test;

import android.util.Log;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.utils.Maths;

/**
 * Created by tarunsingh on 2/17/17.
 */

@TeleOp(name="IMU Test")
public class TwoIMUTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        KingArthur robot = new KingArthur();
        //ElapsedTime runtime = new ElapsedTime();
        AdafruitBNO055IMU imu2;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu2 = (AdafruitBNO055IMU) hardwareMap.get("imu2");//Angle 1 is yaw, angle 2 is roll, angle 3 is pitch
        imu2.initialize(parameters);
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            double imu1Angle = Maths.radiansToDegrees(-robot.imu.getAngularOrientation().firstAngle);
            double imu2Angle = Maths.radiansToDegrees(-imu2.getAngularOrientation().firstAngle);
            sleep(1);
            telemetry.addData("IMU 1", imu1Angle);
            telemetry.addData("IMU 2", imu2Angle);
            telemetry.update();
            Log.d("imutest", imu1Angle + " " + imu2Angle);
        }
    }
}
