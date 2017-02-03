package org.ftcteam5206.subsystems.vision;

import android.app.Activity;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceView;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.imgproc.Moments;

/**
 * Created by tarunsingh on 1/28/17.
 */

public class VisionSystem implements CameraBridgeViewBase.CvCameraViewListener2 {
    private static String TAG = "VisionSystem";

    //OpenCV camera preview object
    private JavaCameraView openCvCameraView;

    private Activity appContext;

    private BaseLoaderCallback baseLoaderCallback = new BaseLoaderCallback(appContext) {
        @Override
        public void onManagerConnected(int status){
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");
                    openCvCameraView.enableView();
                }
                break;
                default:
                {
                    super.onManagerConnected(status);
                }
                break;
            }
        }
    };

    public enum ProcessingMode {
        NONE, BEACON, VORTEX
    }

    private ProcessingMode currentProcessingMode = ProcessingMode.NONE;

    //System time when vision detection is called (for performance analysis)
    public static long lastFrameRequestedTime = 0;

    //0 is rear, 1 is front
    private int cameraId = 0;

    public VisionCallback visionCallback;

    public VisionSystem(OpMode opMode) {
        Log.d(TAG, "Called vision constructor");
        this.appContext = (Activity) opMode.hardwareMap.appContext;
        openCvCameraView = (JavaCameraView) appContext.findViewById(R.id.openCvView);
        openCvCameraView.setVisibility(SurfaceView.VISIBLE);
        openCvCameraView.setCvCameraViewListener(this);

        //Have to use either 3.1.0 or 2.4.13 since Imgproc.moments() isn't in 3.0.0
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appContext, baseLoaderCallback); //Not sure this is actually doing anything
    }

    public void onPause() {
        if(openCvCameraView != null)
            openCvCameraView.disableView();
    }

    public void onResume() {
        //Have to use either 3.1.0 or 2.4.13 since Imgproc.moments() isn't in 3.0.0
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appContext, baseLoaderCallback);
    }

    public void onDestroy() {
        if(openCvCameraView != null)
            openCvCameraView.disableView();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    //Image comes in rotated 90 degrees
    //TODO: Change vision algorithm to avoid unnecessary matrix transformations
    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgb = inputFrame.rgba();
        Mat rgbT = new Mat();
        //Rotate image to be right side up
        Core.transpose(rgb, rgbT);
        Core.flip(rgbT, rgbT, 1);
        Imgproc.resize(rgbT, rgbT, rgb.size());
        switch(currentProcessingMode) {
            case BEACON:
                updateBeaconResult(VisionHelper.detectBeacon(rgbT));
                break;
            case VORTEX:
                //break;
            case NONE:
                return rgbT;
        }

        //VisionSystem detection was called
        if(currentProcessingMode == ProcessingMode.BEACON || currentProcessingMode == ProcessingMode.VORTEX){
            currentProcessingMode = ProcessingMode.NONE;
            //Log how long vision detection took
            Log.i(TAG, "Returned processed frame: " + (System.currentTimeMillis() - lastFrameRequestedTime));
        }
        return rgbT;
    }

    /** Runs beacon detection on next frame */
    public VisionCallback detectBeacon() {
        currentProcessingMode = ProcessingMode.BEACON;
        lastFrameRequestedTime = System.currentTimeMillis();
        visionCallback = new VisionCallback();
        return visionCallback;
    }

    /** Updates vision callback object with result of beacon detection */
    private void updateBeaconResult(double[][] result) {
        visionCallback.update(result[0][0], result[1][0]);
    }

    /** Changes camera being used from rear to front, or front to rear */
    private void swapCamera(){
        //lol kyler would be so proud
        cameraId = cameraId^1;
        openCvCameraView.disableView();
        openCvCameraView.setCameraIndex(cameraId);
        openCvCameraView.enableView();
    }
}
