package org.ftcteam5206.teleop;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;

/**
 * Created by tarunsingh on 3/4/17.
 */

@TeleOp(name="Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        KingArthur robot = new KingArthur();
        ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            //Log.d("autodrive", Boolean.toString(robot.beaconSensor == null));

            telemetry.addData("Clear", robot.beaconSensor.alpha());
            telemetry.addData("Red", robot.beaconSensor.red());
            telemetry.addData("Green", robot.beaconSensor.green());
            telemetry.addData("Blue", robot.beaconSensor.blue());

            telemetry.update();
        }
    }
}
