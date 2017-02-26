package org.ftcteam5206.subsystems.vision;

import android.os.Environment;
import android.util.Log;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.Core;
import org.opencv.imgproc.Moments;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class VisionHelper {
    private static final String TAG = "Vision";

    /** Returns coordinates of center of blue and red mass */
    public static double[][] detectBeacon(Mat src) {
        Log.d(TAG, "Starting actual beacon detection");
        //matrices for images
        Mat HSV = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();
        Mat redHierarchy = new Mat();
        Mat blueHierarchy = new Mat();

        //constants for thresholding
        final int redL = 148;
        final int redH = 178;
        final int blueL = 63;
        final int blueH = 110;
        final int saturation = 100;
        final int value = 100;

        Imgproc.cvtColor(src, HSV, Imgproc.COLOR_RGB2HSV);

        //create matrices with the red and blue areas thresholded
        Core.inRange(HSV, new Scalar(redL,60,value),new Scalar(redH, 200,255),red);
        Core.inRange(HSV, new Scalar(blueL,saturation,value),new Scalar(blueH,255,255),blue);

        saveFrame(red);
        Log.d(TAG, "saved red frame");
        saveFrame(blue);
        Log.d(TAG, "saved blue frame");

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
        double redCenterX = 0;
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
            redCenterX = redMoments.m10/redMoments.m00;
        }

        //repeat previous process, but for blue
        //int blueSelector = 0;

        double blueCenterX = 0;
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
            blueCenterX = blueMoments.m10/blueMoments.m00;
        }

        //draw the contours on the image
        //Imgproc.drawContours(src,redContourList,redSelector,new Scalar(255,0,0),-1);
        //Imgproc.drawContours(src,blueContourList,blueSelector,new Scalar(0,0,255),-1);

        Log.i(TAG, "Red: " + redCenterX + ", Blue: " + blueCenterX);
        Log.i(TAG, "Finished beacon detection: " + (System.currentTimeMillis() - VisionSystem.lastFrameRequestedTime) + "ms");

        double redCenterY = redMoments.m01/redMoments.m00;
        double blueCenterY = blueMoments.m01/blueMoments.m00;

        Imgproc.circle(src, new Point(blueCenterX, blueCenterY), 50, new Scalar(0, 0, 255));
        Imgproc.circle(src, new Point(redCenterX, redCenterY), 50, new Scalar(255, 0, 0));
        saveFrame(src);

        return new double[][]{{redCenterX, redCenterY},{blueCenterX, blueCenterY}};
    }

    /**Returns x and y coordinate of center of vortex of color allianceColor */
    //public static double[] detectVortex(Mat src, boolean isRed) {
    public static Mat detectVortex(Mat src, boolean isRed){
        //matrices for images
        Mat HSV = new Mat();
        Mat mask = new Mat();
        Mat hierarchy = new Mat();

        //constants for thresholding
        final int redL = 200;
        final int redH = 255;
        final int blueL = 125;
        final int blueH = 200;
        final int saturation = 0;
        final int value = 100;

        Log.d(TAG, "Starting vortex detection");
        Imgproc.cvtColor(src, HSV, Imgproc.COLOR_RGB2HSV_FULL);

        if (isRed)
            Core.inRange(HSV, new Scalar(redL, saturation, value), new Scalar(redH, 255, 255), mask);
        else
            Core.inRange(HSV, new Scalar(blueL, saturation, value), new Scalar(blueH, 255, 255), mask);

        List<MatOfPoint> contourList = new ArrayList<>();
        Imgproc.findContours(mask, contourList, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_NONE);

        //Find contour with max perimeter
        MatOfPoint maxPerimeterContour = new MatOfPoint();
        int maxPerimeterIndex = 0;
        if(contourList.size() != 0) {
            MatOfPoint2f contour2f = new MatOfPoint2f();
            contourList.get(maxPerimeterIndex).convertTo(contour2f, CvType.CV_32F);
            double maxPerimeter = Imgproc.arcLength(contour2f, true);
            for (int i = 1; i < contourList.size(); i++) {
                contourList.get(i).convertTo(contour2f, CvType.CV_32F);
                if (Imgproc.arcLength(contour2f, true) > maxPerimeter) {
                    maxPerimeterIndex = i;
                    maxPerimeter = Imgproc.arcLength(contour2f, true);
                }
            }
            maxPerimeterContour = contourList.get(maxPerimeterIndex);
            Log.i(TAG, "Max perimeter: " + maxPerimeter);
        }

        Moments moments = Imgproc.moments(maxPerimeterContour);
        Point center = new Point(moments.m10/moments.m00, moments.m01/moments.m00);

        Log.i(TAG, "Found center of vortex: " + (System.currentTimeMillis() - VisionSystem.lastFrameRequestedTime) + "ms");

        Imgproc.drawContours(mask, contourList, maxPerimeterIndex, new Scalar(255, 255, 255));
        Imgproc.circle(mask, center, 50, new Scalar(255, 255, 255));

        saveFrame(mask);

        //return new double[]{center.x, center.y};
        return mask;
    }

    /** Saves camera frame to internal storage */
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

    /** Returns true if beacon is red */
    public static boolean checkBeacon (Mat src) {
        Mat HSV = new Mat();
        Mat red = new Mat();
        Mat blue = new Mat();

        //constants for thresholding
        final int redL = 148;
        final int redH = 178;
        final int blueL = 63;
        final int blueH = 110;
        final int saturation = 100;
        final int value = 100;

        Imgproc.cvtColor(src, HSV, Imgproc.COLOR_RGB2HSV);

        //create matrices with the red and blue areas thresholded
        Core.inRange(HSV, new Scalar(redL,60,value),new Scalar(redH, 200,255),red);
        Core.inRange(HSV, new Scalar(blueL,saturation,value),new Scalar(blueH,255,255),blue);

        Moments redMoments = Imgproc.moments(red);
        Moments blueMoments = Imgproc.moments(blue);

        if (redMoments.m00 > blueMoments.m00)
            return true;
        else
            return false;
    }
}
