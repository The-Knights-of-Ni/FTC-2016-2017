package org.ftcteam5206.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by tarunsingh on 1/22/17.
 */

@Autonomous(name="OpenCVTestOpMode")
public class OpenCVTestOpMode extends LinearOpMode {
    private static final String TAG = "OpenCV Test";

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        Log.d(TAG, "Starting OpMode");
        OpenCVTest2 cvTest = new OpenCVTest2(this);
        while(opModeIsActive()) {
            telemetry.addData("Status", "Successfully started OpMode");
            telemetry.update();
        }
    }
}
