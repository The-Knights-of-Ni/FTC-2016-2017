package org.ftcteam5206.auto;

/**
 * Created by tarunsingh on 10/30/16.
 */

import android.app.Activity;
import android.util.Log;
import android.widget.SeekBar;
import android.widget.TextView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

@Autonomous(name = "OpenCVTest", group = "Test Code")
public class OpenCVTest extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2 {
    private CameraBridgeViewBase OpenCvCameraView;
    private Mat rgba;
    //private Mat blurredRgba;
    private Mat hsv;
    private Mat mask;
    //private Mat dilatedMask;
    private Mat hierarchy;
    //private Mat contours;

    private SeekBar lowerThreshold;
    private SeekBar upperThreshold;
    private SeekBar perimeterThreshold;
    private TextView lowerThresholdText;
    private TextView upperThresholdText;
    private TextView perimeterThresholdText;

    @Override
    public void runOpMode() {
        BaseLoaderCallback loaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        OpenCvCameraView.enableView();
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        OpenCvCameraView = (CameraBridgeViewBase) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView);
        OpenCvCameraView.setVisibility(CameraBridgeViewBase.VISIBLE);
        OpenCvCameraView.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
        OpenCvCameraView.setCvCameraViewListener(this);

        lowerThreshold = (SeekBar) ((Activity) hardwareMap.appContext).findViewById(R.id.lowerThreshold);
        upperThreshold = (SeekBar) ((Activity) hardwareMap.appContext).findViewById(R.id.upperThreshold);
        perimeterThreshold = (SeekBar) ((Activity) hardwareMap.appContext).findViewById(R.id.perimeterThreshold);
        lowerThresholdText = (TextView) ((Activity) hardwareMap.appContext).findViewById(R.id.lowerThresholdText);
        upperThresholdText = (TextView) ((Activity) hardwareMap.appContext).findViewById(R.id.upperThresholdText);
        perimeterThresholdText = (TextView) ((Activity) hardwareMap.appContext).findViewById(R.id.perimeterThresholdText);


        lowerThreshold.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                lowerThresholdText.setText(String.valueOf(progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        upperThreshold.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                upperThresholdText.setText(String.valueOf(progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        perimeterThreshold.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                perimeterThresholdText.setText(String.valueOf(progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        lowerThreshold.setMax(179);
        upperThreshold.setMax(179);
        perimeterThreshold.setMax(1400);
        upperThreshold.setProgress(165);
        lowerThreshold.setProgress(162);
        perimeterThreshold.setProgress(700);

        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, hardwareMap.appContext, loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }

        try {
            waitForStart();
            while (opModeIsActive()) {
                idle();
            }
        } catch (InterruptedException e){
            //TODO: Error handling
        }

        if (OpenCvCameraView != null) {
            OpenCvCameraView.disableView();
        }
    }



    @Override
    public void onCameraViewStarted(int width, int height) {
        rgba = new Mat(height, width, CvType.CV_8UC4);
        hsv = new Mat();
        mask = new Mat();
        //dilatedMask = new Mat();
        //blurredRgba = new Mat();
        hierarchy = new Mat();
        //contours = new Mat();
    }

    @Override
    public void onCameraViewStopped() {
        if(rgba != null) rgba.release();
    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        rgba.release();
        rgba = inputFrame.rgba();
        //Mat rgbaT = rgba.t();
        //Core.flip(rgba.t(), rgbaT, 1);
        Imgproc.resize(rgba, rgba, rgba.size());
        Imgproc.cvtColor(rgba, hsv, Imgproc.COLOR_RGB2HSV_FULL);

        int lowHsvValue = lowerThreshold.getProgress();
        int highHsvValue = upperThreshold.getProgress();
        int minimumPerimeter = perimeterThreshold.getProgress();

        Core.inRange(hsv, new Scalar(lowHsvValue, 0, 0), new Scalar(highHsvValue, 255, 255), mask);

        List<MatOfPoint> contourList = new ArrayList<>();
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        //lowerThresholdText.setText(String.valueOf(contourList.size()) + " contours");

        //Find contour with max area
        /*int maxAreaIndex = 0;
        if(contourList.get(maxAreaIndex) == null)
            return mask;
        double maxArea = Imgproc.contourArea(contourList.get(maxAreaIndex));
        for(int i = 1; i < contourList.size(); i++){
            MatOfPoint contour = contourList.get(i);
            if(contour != null){
                if(Imgproc.contourArea(contour) > maxArea){
                    maxAreaIndex = i;
                    maxArea = Imgproc.contourArea(contourList.get(maxAreaIndex));
                }
            }
        } */

        //Filter contours for minimum perimeter
        Iterator<MatOfPoint> contourIterator = contourList.iterator();
        while(contourIterator.hasNext()){
            if(Imgproc.arcLength(new MatOfPoint2f(contourIterator.next().toArray()), true) < minimumPerimeter){
                contourIterator.remove();
            }
        }

        Log.d("debug", "contourList is " + String.valueOf(contourList != null));
        //Imgproc.drawContours(mask, contourList, -1, new Scalar(255, 255, 255));
        return mask;
    }
}