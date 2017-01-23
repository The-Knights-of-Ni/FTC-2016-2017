package org.ftcteam5206.auto;

import android.app.Activity;
import android.os.Environment;
import android.util.Log;
import android.os.Bundle;
import android.view.SurfaceView;
import android.view.View;
import android.widget.Button;
import android.widget.RadioButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.util.ArrayList;

import org.firstinspires.ftc.teamcode.R;
/**
 * Created by tarunsingh on 12/18/16.
 */

public class OpenCVTest2 implements CvCameraViewListener2 {
    private static final String TAG = "OpenCV Test";
    private static Activity appContext;
    private static Telemetry telemetry;

    //OpenCV camera preview object
    private CameraBridgeViewBase openCvCameraView;

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

    public enum AllianceColor {
        RED, BLUE
    }

    private ProcessingMode currentProcessingMode = ProcessingMode.NONE;

    private AllianceColor allianceColor;

    //System time when vision detection is called (for performance analysis)
    public static long lastFrameRequestedTime = 0;

    //0 is rear, 1 is front
    private int cameraId = 0;

    //Stores result of vision detection to display on UI
    public static String result;

    //TextView to display result of beacon detection
    private TextView resultTextView;
    private Button detectBeaconButton;
    private Button detectVortexButton;

    private SeekBar lowHueSeekBar;
    private SeekBar highHueSeekBar;

    public static double lowHue;
    public static double highHue;

    public OpenCVTest2(OpMode opMode) {
        Log.d(TAG, "Called constructor");
        this.appContext = (Activity) opMode.hardwareMap.appContext;
        this.telemetry = opMode.telemetry;
        init();
    }

    private void init() {
        Log.d(TAG, "Called init");
        openCvCameraView = (CameraBridgeViewBase) appContext.findViewById(R.id.openCvView);
        openCvCameraView.setVisibility(SurfaceView.VISIBLE);
        openCvCameraView.setCvCameraViewListener(this);
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, appContext, baseLoaderCallback);

        detectBeaconButton = (Button) appContext.findViewById(R.id.detectBeaconButton);
        detectBeaconButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                startBeaconDetection();
            }
        });

        lowHueSeekBar = (SeekBar) appContext.findViewById(R.id.lowValueSeekBar);
        highHueSeekBar = (SeekBar) appContext.findViewById(R.id.highValueSeekBar);
        lowHueSeekBar.setMax(180);
        highHueSeekBar.setMax(180);

        lowHueSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                lowHue = progress;
                Log.d(TAG, "Low value: " + progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });

        highHueSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                highHue = progress;
                Log.d(TAG, "High value: " + progress);
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
    }

    /*
    @Override
    public Mat onCameraFrame(CvCameraViewFrame frame) {return frame.rgba();}
    */
    /*
    @Override
    public void onCreate(Bundle savedInstanceState) {
        Log.d(TAG, "Called onCreate");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main_layout);
        openCvCameraView = (CameraBridgeViewBase) findViewById(R.id.openCvView);
        openCvCameraView.setVisibility(SurfaceView.VISIBLE);
        openCvCameraView.setCvCameraViewListener(this);

        resultTextView = (TextView) findViewById(R.id.resultTextView);

        detectBeaconButton = (Button) findViewById(R.id.detectBeaconButton);
        detectBeaconButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                startBeaconDetection();
            }
        });

        detectVortexButton = (Button) findViewById(R.id.detectVortexButton);
        detectVortexButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                startVortexDetection();
            }
        });

        lowHueSeekBar = (SeekBar) findViewById(R.id.lowHue);
        highHueSeekBar = (SeekBar) findViewById(R.id.highHue);
        lowHueSeekBar.setMax(180);
        highHueSeekBar.setMax(180);
        lowHueSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                lowHue = i;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        highHueSeekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
                highHue = i;
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {

            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }
        });
        //Have to use either 3.1.0 or 2.4.13 since Imgproc.moments() isn't in 3.0.0
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, baseLoaderCallback);

        ArrayList<View> views = new ArrayList<>();
        views.add(detectBeaconButton);
        views.add(detectVortexButton);
        openCvCameraView.addTouchables(views);
    }
    */
    /*
    public void onPause() {
        super.onPause();
        if(openCvCameraView != null)
            openCvCameraView.disableView();
    }

    public void onResume() {
        super.onResume();
        //Have to use either 3.1.0 or 2.4.13 since Imgproc.moments() isn't in 3.0.0
        OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_1_0, this, baseLoaderCallback);
    }

    public void onDestroy() {
        super.onDestroy();
        if(openCvCameraView != null)
            openCvCameraView.disableView();
    }
    */
    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    //Image comes in rotated 90 degrees
    //TODO: Change vision algorithm to avoid unnecessary matrix transformations
    @Override
    public Mat onCameraFrame(CvCameraViewFrame inputFrame) {
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
            case VORTEX:
                //detectVortex(rgb);
                break;
            case NONE:
                return rgbT;
        }

        //Vision detection was called
        if(currentProcessingMode == ProcessingMode.BEACON || currentProcessingMode == ProcessingMode.VORTEX){
            //currentProcessingMode = ProcessingMode.NONE;
            /*
            //Update UI with results of vision detection
            Log.i(TAG, result);
            //TODO: Probably want to switch to AsyncTask instead
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    updateResult(result);
                }
            });
            //Log how long vision detection took
            //Log.i(TAG, "Returned processed frame: " + (System.currentTimeMillis() - lastFrameRequestedTime));
            */
        }

        return rgbT;
    }

    /** Runs beacon detection on next frame */
    private void startBeaconDetection() {
        currentProcessingMode = ProcessingMode.BEACON;
        lastFrameRequestedTime = System.currentTimeMillis();
        Log.d(TAG, "Called startBeaconDetection");
    }

    private void detectBeacon(Mat rgb) {
        boolean redIsRight = VisionHelper.detectBeacon(rgb);
        if(redIsRight)
            result = "Blue is left, red is right";
        else
            result = "Red is left, blue is right";
    }

    /** Runs vortex detection on next frame */
    private void startVortexDetection() {
        currentProcessingMode = ProcessingMode.VORTEX;
        lastFrameRequestedTime = System.currentTimeMillis();
    }
    /*
    private void detectVortex(Mat rgb) {
        //Log.i(TAG, "Called detectVortex(Mat rgb)");
        double[] center = VisionHelper.detectVortex(rgb, allianceColor);
        result = "X: " + center[0] + ", Y: " + center[1];
    }
    */
    /** Changes camera being used from rear to front, or front to rear */
    private void swapCamera(){
        //lol kyler would be so proud
        cameraId = cameraId^1;
        openCvCameraView.disableView();
        openCvCameraView.setCameraIndex(cameraId);
        openCvCameraView.enableView();
    }

    /** Wrapper method to update UI */
    private void updateResult(String result) {
        resultTextView.setText(result);
    }
    /*
    public void onRadioButtonClicked(View view) {
        boolean checked = ((RadioButton) view).isChecked();
        switch(view.getId()) {
            case R.id.allianceColorRed:
                if(checked)
                    allianceColor = AllianceColor.RED;
                break;
            case R.id.allianceColorBlue:
                if(checked)
                    allianceColor = AllianceColor.BLUE;
        }
    }
    */
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
