package org.ftcteam5206.teleop;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.utils.Button.ButtonHandler;

/**
 * Created by tarunsingh on 3/4/17.
 */

@TeleOp(name="Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        KingArthur robot = new KingArthur();
        ElapsedTime runtime = new ElapsedTime();
        ButtonHandler pad1 = new ButtonHandler();
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            //Log.d("autodrive", Boolean.toString(robot.beaconSensor == null));
            try {
                pad1.updateButtons(gamepad1.toByteArray(), gamepad1.left_trigger, gamepad1.right_trigger);
                //pad2.updateButtons(gamepad2.toByteArray(), gamepad2.left_trigger, gamepad2.right_trigger);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            telemetry.addData("Clear", robot.beaconSensor.alpha());
            telemetry.addData("Red", robot.beaconSensor.red());
            telemetry.addData("Green", robot.beaconSensor.green());
            telemetry.addData("Blue", robot.beaconSensor.blue());
            float[] hsv = new float[3];
            Color.RGBToHSV(robot.beaconSensor.red(), robot.beaconSensor.green(), robot.beaconSensor.blue(), hsv);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Saturation", hsv[1]);
            telemetry.addData("Value", hsv[2]);
            telemetry.update();

            if(pad1.singlePress(pad1.buttons.X)){
                if(hsv[0] > 200 && hsv[0] < 230){
                    //Blue on Left

                }
                else if((hsv[0] > 0 && hsv[0] < 30) || (hsv[0] > 350 && hsv[0] <= 360)){
                    //Red on Left
                }
                else {
                    //skip beacon
                }

            }

        }
    }
}
