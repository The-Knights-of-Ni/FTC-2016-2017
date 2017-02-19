package org.ftcteam5206.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.ftcteam5206.KingArthur;

/**
 * Created by tarunsingh on 2/4/17.
 */
@Disabled
@TeleOp(name="PotTest")
public class PotTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput pot = (AnalogInput) hardwareMap.get("pot");
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("Pot Volt", pot.getVoltage());
            telemetry.update();
        }
    }
}
