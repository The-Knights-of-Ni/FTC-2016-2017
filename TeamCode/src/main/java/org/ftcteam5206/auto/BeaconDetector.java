package org.ftcteam5206.auto;

import org.opencv.core.Core;
import org.opencv.core.Mat;

import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import static android.R.attr.value;
import static android.os.Build.VERSION_CODES.M;

/**
 * Created by tarunsingh on 12/10/16.
 */

public class BeaconDetector {
    public static boolean blueIsOnRight = false;
    public static boolean hasBeenTriggered = false;

    //constants for thresholding
    private final static int redL = 200;
    private final static int redH = 255;
    private final static int blueL = 125;
    private final static int blueH = 200;
    private final static int saturation = 0;
    private final static int value = 100;

    public static void detectBeacon(byte[] data){
        Log.d("beacon", "checkpoint 1");
        Mat HSV = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();
        Mat redHierarchy = new Mat();
        Mat blueHierarchy = new Mat();

        Log.d("beacon", "checkpoint 2");
        Mat yuv = new Mat(data.length, data.length, CvType.CV_8UC3);
        yuv.put(0, 0, data);
        Imgproc.resize(yuv,yuv,yuv.size());
        Imgproc.cvtColor(yuv,HSV,Imgproc.COLOR_YUV2RGB);

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
        if(redCenter > blueCenter)
            blueIsOnRight = true;
        else
            blueIsOnRight = false;
        hasBeenTriggered = true;
    }
}
