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
                telemetry.addData("Pad 1", pad1.updateButtons(gamepad1.toByteArray(), gamepad1.left_trigger, gamepad1.right_trigger));
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            try {
                telemetry.addData("Pad 2", pad2.updateButtons(gamepad2.toByteArray(), gamepad2.left_trigger, gamepad2.right_trigger));
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            if(pad1.singlePress(pad1.buttons.A)) pad1.setHairTriggerLevel(0.9);
            if(pad1.singlePress(pad1.buttons.X)) pad1.setHairTriggerLevel(0.1);

            telemetry.addData("Pad 1 RT", pad1.rightTriggerCurrent);
            telemetry.addData("Pad 1 RT Tog", pad1.toggle(pad1.buttons.RIGHT_TRIGGER));
            telemetry.addData("Pad 2 RT", pad2.rightTriggerCurrent);
            telemetry.addData("Pad 2 RT Tog", pad2.toggle(pad2.buttons.RIGHT_TRIGGER));
            telemetry.addData("Pad 1 Hair Trigger", pad1.hairTrigger);


            telemetry.update();
        }
    }
}
