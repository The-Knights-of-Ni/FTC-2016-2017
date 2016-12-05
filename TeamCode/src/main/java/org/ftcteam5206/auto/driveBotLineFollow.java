/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.ftcteam5206.auto;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.ftcteam5206.hardwareDriveBot;

@Autonomous(name="DriveBot:Line Follow", group="DriveBot")
public class driveBotLineFollow extends LinearOpMode {

    hardwareDriveBot robot   = new hardwareDriveBot();
    private ElapsedTime runtime = new ElapsedTime();

    static final double insideSpeed = 0;
    static final double outsideSpeed = 0.06;
    static  final double fSpeed = 0.1;
    double raw = 0;
    double edited = 0;
    float hsvValues[] = {0F,0F,0F};

    @Override
    public void runOpMode() throws InterruptedException {

        //setup hardware
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //follow teh line forwards
        while(opModeIsActive() && raw < 0.01)
        {
            //try to find object in front
            raw = robot.odsSensor.getRawLightDetected();
            edited = robot.odsSensor.getLightDetected();

            //get the color the sensor is seeing
            Color.RGBToHSV(robot.colorSensor.red()*8,robot.colorSensor.green()*8,robot.colorSensor.blue()*8,hsvValues);
            //if looking at the white line
            if(hsvValues[0] < 100)
            {
                //left side is outside, right is inside
                robot.lfMotor.setPower(outsideSpeed);
                robot.lbMotor.setPower(outsideSpeed);
                robot.rfMotor.setPower(insideSpeed);
                robot.rbMotor.setPower(insideSpeed);
            }
            //if looking at the mat
            else
            {
                //left motors are inside, right are outside
                robot.lfMotor.setPower(insideSpeed);
                robot.lbMotor.setPower(insideSpeed);
                robot.rfMotor.setPower(outsideSpeed);
                robot.rbMotor.setPower(outsideSpeed);
            }
            telemetry.addData("Hue", "%.2f",hsvValues[0]);
            telemetry.addData("Raw", "%.2f",raw);
            telemetry.addData("Edited", "%.2f",edited);
            telemetry.update();
        }

        //drive forward and bump the beacon
        robot.lbMotor.setPower(fSpeed);
        robot.lfMotor.setPower(fSpeed);
        robot.rbMotor.setPower(fSpeed);
        robot.rfMotor.setPower(fSpeed);

        sleep(1000);

        //stop the robot
        robot.lbMotor.setPower(0);
        robot.lfMotor.setPower(0);
        robot.rbMotor.setPower(0);
        robot.rfMotor.setPower(0);

        telemetry.addData("Say", "Complete");
        telemetry.update();
    }
}

