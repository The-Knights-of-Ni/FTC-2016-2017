package org.ftcteam5206.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.RobotConstants;

/**
 * Created by tarunsingh on 2/25/17.
 */

@TeleOp(name="RPM Test")
public class RPMTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        KingArthur robot = new KingArthur();
        ElapsedTime runtime = new ElapsedTime();
        robot.init(hardwareMap);
        waitForStart();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 10) {robot.launcher.setPower(1);}
        double startTicks = robot.launcher.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 30) {robot.launcher.setPower(1);}
        robot.launcher.setPower(0);
        double changeInTicks = robot.launcher.getCurrentPosition() - startTicks;
        double elapsedTime = runtime.seconds();
        double revs = changeInTicks / (RobotConstants.neverRestCPR*RobotConstants.neverRestPPR*3);
        double mins = elapsedTime/60;
        double rpm = revs/mins;
        while (opModeIsActive()) {
            telemetry.addData("Average RPM", rpm);
            telemetry.update();
        }
    }
}
