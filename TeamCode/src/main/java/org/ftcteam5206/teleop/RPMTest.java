package org.ftcteam5206.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ftcteam5206.KingArthur;
import org.ftcteam5206.subsystems.Launcher;
import org.ftcteam5206.subsystems.RobotConstants;

/**
 * Created by tarunsingh on 2/25/17.
 */
@Disabled
@TeleOp(name="RPM Test")
public class RPMTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor launcher1 = hardwareMap.dcMotor.get("launcher");
        DcMotor launcher2 = hardwareMap.dcMotor.get("launcher2");
        ElapsedTime runtime = new ElapsedTime();
        Launcher launcher = new Launcher(launcher1, launcher2, null, null, runtime);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            launcher.setPower(1);
            //launcher.setRPM(2400);
            telemetry.addData("Launcher RPM", launcher.getRPM());
            telemetry.addData("Launcher PWM", launcher.launcher.getPower());
            telemetry.addData("RPM Error", launcher.error);
            telemetry.update();
        }
    }
}
