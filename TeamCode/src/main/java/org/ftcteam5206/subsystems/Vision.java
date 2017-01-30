package org.ftcteam5206.subsystems;

import android.app.Activity;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceView;
import android.widget.RadioButton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ftcteam5206.auto.Mk2Auto;
import org.ftcteam5206.auto.OpenCVTest2;
import org.ftcteam5206.teleop.Mk2Teleop;
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

public class Vision implements CameraBridgeViewBase.CvCameraViewListener2 {
    private static String TAG = "Vision";

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

    public Vision(OpMode opMode) {
        Log.d(TAG, "Called vision constructor");
        this.appContext = (Activity) opMode.hardwareMap.appContext;
        openCvCameraView = (JavaCameraView) appContext.findViewById(R.id.openCvView);
        openCvCameraView.setVisibility(SurfaceView.VISIBLE);
        openCvCameraView.setCvCameraViewListener(this);

        //Have to use either 3.1.0 or 2.4.13 since Imgproc.moments() isn't in 3.0.0
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appContext, baseLoaderCallback);
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
                detectBeacon(rgbT);
                break;
            case NONE:
                return rgbT;
        }

        //Vision detection was called
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
        VisionCallback visionCallback = new VisionCallback();
        this.visionCallback = visionCallback;
        return visionCallback;
    }

    private void detectBeacon(Mat rgb) {
        Log.d(TAG, "Starting actual beacon detection");
        //matrices for images
        Mat HSV;
        Mat red;
        Mat blue;
        Mat redHierarchy;
        Mat blueHierarchy;

        //constants for thresholding
        final int redL = 152;
        final int redH = 180;
        final int blueL = 88;
        final int blueH = 118;
        final int saturation = 0;
        final int value = 0;

        HSV = new Mat();
        red = new Mat();
        blue = new Mat();
        redHierarchy = new Mat();
        blueHierarchy = new Mat();

        Imgproc.cvtColor(rgb, HSV, Imgproc.COLOR_RGB2HSV);

        //create matrices with the red and blue areas thresholded
        Core.inRange(HSV, new Scalar(redL,saturation,value),new Scalar(redH, 255,255),red);
        Core.inRange(HSV, new Scalar(blueL,saturation,value),new Scalar(blueH,255,255),blue);

        //return red;

        //store contour hierarchies for thresholded stuff
        List<MatOfPoint> redContourList = new ArrayList<>();
        List<MatOfPoint> blueContourList = new ArrayList<>();

        //find the contours in each thresholded image
        Imgproc.findContours(red, redContourList, redHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);
        Imgproc.findContours(blue, blueContourList, blueHierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        //find the largest contours, then place them on top of the source image
        //also find the center of the blob of redness
        //if there are no red contours, skip this step
        Moments redMoments = Imgproc.moments(red);
        double redCenter = 0;
        //int redSelector = 0;
        if(redContourList.size() != 0) {
        /*
        //find the largest red contour
        double maxArea = Imgproc.contourArea(redContourList.get(redSelector));
        for (int i = 1; i < redContourList.size(); i++) {
            MatOfPoint redContour = redContourList.get(i);
            if (redContour != null) {
                if (Imgproc.contourArea(redContourList.get(i)) > maxArea) {
                    redSelector = i;
                    maxArea = Imgproc.contourArea(redContourList.get(redSelector));
                }
            }
        }
        */
            redCenter = redMoments.m10/redMoments.m00;
        }

        //repeat previous process, but for blue
        //int blueSelector = 0;

        double blueCenter = 0;
        Moments blueMoments = Imgproc.moments(blue);
        if(blueContourList.size() != 0) {
        /*
        //find the largest blue contour
        double maxArea = Imgproc.contourArea(blueContourList.get(blueSelector));
        for (int i = 1; i < blueContourList.size(); i++) {
            MatOfPoint redContour = blueContourList.get(i);
            if (redContour != null) {
                if (Imgproc.contourArea(blueContourList.get(i)) > maxArea) {
                    blueSelector = i;
                    maxArea = Imgproc.contourArea(blueContourList.get(blueSelector));
                }
            }
        }
        */
            blueCenter = blueMoments.m10/blueMoments.m00;
        }

        //draw the contours on the image
        //Imgproc.drawContours(src,redContourList,redSelector,new Scalar(255,0,0),-1);
        //Imgproc.drawContours(src,blueContourList,blueSelector,new Scalar(0,0,255),-1);

        Log.i(TAG, "Red: " + redCenter + ", Blue: " + blueCenter);
        Log.i(TAG, "Finished beacon detection: " + (System.currentTimeMillis() - lastFrameRequestedTime) + "ms");

        Imgproc.circle(rgb, new Point(blueCenter, blueMoments.m01/blueMoments.m00), 50, new Scalar(0, 0, 255));
        Imgproc.circle(rgb, new Point(redCenter, redMoments.m01/redMoments.m00), 50, new Scalar(255, 0, 0));
        saveFrame(rgb);
        visionCallback.update(blueCenter, redCenter);
    }

    /** Changes camera being used from rear to front, or front to rear */
    private void swapCamera(){
        //lol kyler would be so proud
        cameraId = cameraId^1;
        openCvCameraView.disableView();
        openCvCameraView.setCameraIndex(cameraId);
        openCvCameraView.enableView();
    }

    public static void saveFrame(Mat img) {
        File path = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        String filename = "FRAME_" + System.currentTimeMillis() + ".png";
        File file = new File(path, filename);
        filename = file.toString();
        boolean success = Imgcodecs.imwrite(filename, img);
        if(success)
            Log.d(TAG, "Successfully saved image");
        else
            Log.d(TAG, "Failed to save image");
        //img.release();
    }
}
