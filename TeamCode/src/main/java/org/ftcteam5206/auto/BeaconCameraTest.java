package org.ftcteam5206.auto;

import android.app.Activity;
import android.hardware.Camera;
import android.os.Bundle;
import android.os.Environment;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.FrameLayout;
import android.hardware.Camera.PictureCallback;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.teamcode.R;
import org.ftcteam5206.auto.CameraPreview;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.R.attr.id;
import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.picture;
import static org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity.pictureHasBeenTaken;

@Autonomous (name="BeaconTest", group="Test Code")
public class BeaconCameraTest extends LinearOpMode {

    Activity context;

    public void runOpMode() {
        Log.d("beacon", "Taking picture from RC");
        FtcRobotControllerActivity.takePicture();
        Log.d("beacon", "waiting for picture to be taken");
        while(!FtcRobotControllerActivity.pictureHasBeenTaken){
            try{
                sleep(10);
            } catch (Exception e){

            }
        }
        Log.d("beacon", "picture has been taken");
        BeaconDetector.detectBeacon(FtcRobotControllerActivity.data);
        while(!BeaconDetector.hasBeenTriggered){
            try {
                sleep(10);
            } catch (Exception e){
            }
        }
        if(!BeaconDetector.hasBeenTriggered){
            Log.d("beacon", "Beacon Detector was not triggered");
        } else {
            if(BeaconDetector.blueIsOnRight){
                Log.d("beacon", "blue is on right, red is on left");
            }
            if(!BeaconDetector.blueIsOnRight){
                Log.d("beacon", "red is on right, blue is on left");
            }
        }
    }
}