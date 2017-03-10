package org.ftcteam5206.auto;

import android.util.Log;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.vision.VisionSystem;

/**
 * Created by Parker on 3/10/17.
 */

@Autonomous(name = "visionTest", group = "Autonomous")
public class visionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //KingArthur robot = new KingArthur();
        //robot.init(hardwareMap);
        VisionSystem visionSystem = new VisionSystem(this);

        waitForStart();

        Log.d("autotest", "Starting");

        visionSystem.findBallLocations();

        Log.d("autotest", "Vision Done");
        Log.d("VisionHelper", "Called visionsystem.detectbeacon() in auto");
        while (opModeIsActive() && !visionSystem.visionCallback.hasFinished) {}

        while(opModeIsActive()) {
            for (double[] x : visionSystem.visionCallback.ballLocationsRed)
                telemetry.addData("ngfjhg", x[0] + " " + x[1]);
            telemetry.update();
        }

    }
}
