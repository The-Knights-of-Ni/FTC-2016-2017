package org.ftcteam5206.auto;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Created by Parker on 12/4/16.
 * Read Tarun's OpenCV test, tried to repurpose it to do beacon color detection
 */

@Disabled
@Autonomous (name = "beaconDetect", group = "Test Code")
public class beaconDetect extends LinearOpMode implements CameraBridgeViewBase.CvCameraViewListener2{

    private CameraBridgeViewBase cameraPreview;

    //matrices for images
    private Mat src;
    private Mat HSV;
    private Mat red;
    private Mat blue;
    private Mat redHierarchy;
    private Mat blueHierarchy;

    //constants for thresholding
    private final static int redL = 200;
    private final static int redH = 255;
    private final static int blueL = 125;
    private final static int blueH = 200;
    private final static int saturation = 0;
    private final static int value = 100;

    @Override
    public void runOpMode() {
        BaseLoaderCallback loaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS: {
                        cameraPreview.enableView();
                    }
                    break;
                    default: {
                        super.onManagerConnected(status);
                    }
                    break;
                }
            }
        };

        //cameraPreview = (CameraBridgeViewBase) ((Activity) hardwareMap.appContext).findViewById(R.id.surfaceView);
        cameraPreview.setVisibility(CameraBridgeViewBase.VISIBLE);
        cameraPreview.setCameraIndex(CameraBridgeViewBase.CAMERA_ID_BACK);
        cameraPreview.setCvCameraViewListener(this);

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
            //lol so shamelessly copied from Tarun
        }

        if (cameraPreview != null) {
            cameraPreview.disableView();
        }

    }
        @Override
        public void onCameraViewStarted(int width, int height) {
            src = new Mat(height, width, CvType.CV_8UC4);
            HSV = new Mat();
            red = new Mat();
            blue = new Mat();
            redHierarchy = new Mat();
            blueHierarchy = new Mat();
        }

        @Override
        public void onCameraViewStopped() {
            if(src != null) src.release();
        }

        @Override
        public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {

            //my debugging methods are always incredibly professional
            //qLog.d("debug", "HIHIHIHIHIHIHIHIHIHIHIHI");
            Log.d("Beacon", "Starting beacon detection");
            src.release();
            src = inputFrame.rgba();

            Imgproc.resize(src,src,src.size());
            Imgproc.cvtColor(src,HSV,Imgproc.COLOR_RGB2HSV_FULL);

            //create matrices with the red and blue areas thresholded
            Core.inRange(HSV, new Scalar(redL,saturation,value),new Scalar(redH, 255,255),red);
            Core.inRange(HSV, new Scalar(blueL,saturation,value),new Scalar(blueH,255,255),blue);

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
            int redSelector = 0;
            if(redContourList.size() != 0) {
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
                redCenter = redMoments.m10/redMoments.m00;
            }

            //repeat previous process, but for blue
            int blueSelector = 0;
            double blueCenter = 0;
            Moments blueMoments = Imgproc.moments(blue);
            if(blueContourList.size() != 0) {
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
                blueCenter = blueMoments.m10/blueMoments.m00;
            }

            //draw the contours on the image
            Imgproc.drawContours(src,redContourList,redSelector,new Scalar(255,0,0),-1);
            Imgproc.drawContours(src,blueContourList,blueSelector,new Scalar(0,0,255),-1);

            //this may need to be changed based upon camera orientation
            //image is assumed to be upside down
            Log.d("Beacon", "Red: " + redCenter + ", Blue: " + blueCenter);
            if(redCenter > blueCenter)
                Log.d("Beacon", "Red is Left, Blue is Right");
            else
                Log.d("Beacon", "Blue is Left, Red is Right");
            cameraPreview.disableView();
            return src;
        }
}
