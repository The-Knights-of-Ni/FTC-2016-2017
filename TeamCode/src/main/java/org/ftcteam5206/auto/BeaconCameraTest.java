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

import org.firstinspires.ftc.teamcode.R;
import org.ftcteam5206.auto.CameraPreview;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;

@Autonomous (name="BeaconTest", group="Test Code")
public class BeaconCameraTest extends LinearOpMode {

    Activity context;

    public void runOpMode() {

    }
}