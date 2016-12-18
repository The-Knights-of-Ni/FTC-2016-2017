package org.ftcteam5206.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;

import org.ftcteam5206.utils.Button.*;

@TeleOp(name="ButtonDebugger", group="Tester")
public class ToggleDebug extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ButtonHandler pad1 = new ButtonHandler();
        ButtonHandler pad2 = new ButtonHandler();

        waitForStart();


        while (opModeIsActive())
        {
            try {
                telemetry.addData("Pad 1", pad1.updateButtons(gamepad1.toByteArray()));
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            try {
                telemetry.addData("Pad 2", pad2.updateButtons(gamepad2.toByteArray()));
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            telemetry.addData("Pad 1 A", pad1.toggle(pad1.buttons.A));
            telemetry.addData("Pad 2 A", pad2.toggle(pad2.buttons.A));

            telemetry.update();
        }
    }
}
