package org.ftcteam5206.auto;

import android.nfc.Tag;
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

        Log.d("autotest", "Vision Done");
        Log.d("VisionHelper", "Called visionsystem.detectbeacon() in auto");

        visionSystem.findBallLocations();
        while (opModeIsActive() && !visionSystem.visionCallback.hasFinished) {}

        visionSystem.disableCamera();
        int counter = 0;
        while(opModeIsActive()) {
            for (int i = 0; i < 3; i ++)
            {
                telemetry.addData("RED:",visionSystem.visionCallback.ballLocations[i][0] + " " + visionSystem.visionCallback.ballLocations[i][1] + " " + visionSystem.visionCallback.ballLocations[i][2]);
            }

            for (int i = 3; i < 6; i ++)
            {
                telemetry.addData("BLUE:",visionSystem.visionCallback.ballLocations[i][0] + " " + visionSystem.visionCallback.ballLocations[i][1] + " " + visionSystem.visionCallback.ballLocations[i][2]);
            }

            counter++;

            telemetry.addData("Thing",counter);
            telemetry.update();
            sleep(1000);
        }
    }
}
