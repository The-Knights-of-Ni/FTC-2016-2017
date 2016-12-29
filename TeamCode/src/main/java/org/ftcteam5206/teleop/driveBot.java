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
package org.ftcteam5206.teleop;

import android.graphics.Color;

import org.ftcteam5206.hardwareDriveBot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.util.Range;


//copied template from PushbotTeleopTank_Interative.java

@TeleOp(name="DriveBot_test", group="DriveBot")
public class driveBot extends OpMode{
    //variables for controlling the robot
    double drive;
    double turn;
    double rPower;
    double lPower;
    int driverMode;
    /*
    double pusher;
    final double servoLeft = 0.1;
    final double servoRight = 0.84;
    float hsvValues[] = {0F,0F,0F};
    double rawValue;
    double nRawValue;
    */

    /* Declare OpMode members. */
    hardwareDriveBot robot = new hardwareDriveBot();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        //robot.colorSensor.enableLed(true);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // Run wheels in not tank mode
        //set motor speeds
        if (gamepad1.left_bumper = true){
            driverMode = 1;
        } else if (gamepad1.right_bumper = true){
            driverMode = 2;
        } else {
            driverMode = 0;
        }


        switch (driverMode){
            //speed mode
            case 1:
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.left_stick_x;
                telemetry.addData("DriveMode", "%.2f", (double)driverMode);
                break;
            //slow mode
            case 2:
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.left_stick_x;
                telemetry.addData("DriveMode", "%.2f", (double)driverMode);
                break;
            //normal mode
            default:
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.left_stick_x;
                telemetry.addData("DriveMode", "%.2f", (double)driverMode);
                break;
        }

        //set motor powers
        rPower = drive - turn/2.5;
        lPower = drive + turn/2.5;

        robot.lfMotor.setPower(lPower);
        robot.lbMotor.setPower(lPower);
        robot.rfMotor.setPower(rPower);
        robot.rbMotor.setPower(rPower);

        /*
        //set the position of the button pusher servo
        if(gamepad1.right_bumper)
            pusher = servoRight;
        else if(gamepad1.left_bumper)
            pusher = servoLeft;
        robot.buttonPusher.setPosition(pusher);

        //get the HSV values from the color sensor
        Color.RGBToHSV(robot.colorSensor.red() * 8, robot.colorSensor.green() * 8, robot.colorSensor.blue() * 8, hsvValues);
        //get light values from ODS
        nRawValue = robot.odsSensor.getLightDetected();
        rawValue = robot.odsSensor.getRawLightDetected();
        */
        // Send telemetry message to signify robot running;
        telemetry.addData("drive",  "%.2f", -drive);
        telemetry.addData("turn", "%.2f", turn);
        //telemetry.addData("rb", gamepad1.right_bumper);
        //telemetry.addData("lb", gamepad1.left_bumper);
        //telemetry.addData("Servo", "%.2f", pusher);
        //telemetry.addData("Hue", "%.2f", hsvValues[0]);
        //telemetry.addData("Sat", "%.2f", hsvValues[1]);
        //telemetry.addData("Val", "%.2f", hsvValues[2]);
        //telemetry.addData("Raw", "%.2f", rawValue);
        //telemetry.addData("nonRaw", "%.2f", nRawValue);

        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Say", "Goodbye");
        updateTelemetry(telemetry);
    }

}
