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
package org.ftcteam5206;

import org.ftcteam5206.hardwareDriveBot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


//copied template from PushbotTeleopTank_Interative.java

@TeleOp(name="DriveBot_test", group="DriveBot")
public class driveBot extends OpMode{
    //variables for controlling the robot
    ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    double lastTime;
    double dTime;
    double drive;
    double turn;
    double rPower;
    double lPower;
    int driverMode;
    boolean intake = false;
    boolean intakelast = false;

    public void Drive(){
        // Run wheels in not tank mode
        //set motor speeds
        if (gamepad1.right_bumper){
            driverMode = 2;
        } else if (gamepad1.left_bumper){
            driverMode = 1;
        } else {
            driverMode = 0;
        }

        switch (driverMode){
            //speed mode
            case 1:
                drive = -gamepad1.left_stick_y;
                turn = gamepad1.left_stick_x/2.3;
                telemetry.addData("DriveMode", "%.2f", (double)driverMode);
                break;
            //slow mode
            case 2:
                drive = -gamepad1.left_stick_y/4;
                turn = gamepad1.left_stick_x/3;
                telemetry.addData("DriveMode", "%.2f", (double)driverMode);
                break;
            //normal mode
            default:
                drive = -gamepad1.left_stick_y/2;
                turn = gamepad1.left_stick_x/2.75;
                telemetry.addData("DriveMode", "%.2f", (double)driverMode);
                break;
        }

        //set motor powers
        rPower = drive - turn;
        lPower = drive + turn;

        robot.lfMotor.setPower(lPower);
        robot.lbMotor.setPower(lPower);
        robot.rfMotor.setPower(rPower);
        robot.rbMotor.setPower(rPower);

        // Send telemetry message to signify robot running;
        telemetry.addData("drive",  "%.2f", -drive);
        telemetry.addData("turn", "%.2f", turn);

    }

    public void Score (){
        //transport and shooter mode
        if(gamepad1.x){
            //shoot
            telemetry.addData("Say", "Shooting");
        } else  if (gamepad1.a){
            //spin up
            telemetry.addData("Say", "Spinning up");
        } else if (gamepad1.b){
            //run transport
            telemetry.addData("Say", "Transporting");
        }
        //code for running the intake
        if (gamepad1.y == true) {
            robot.intake.setPosition(1);
            telemetry.addData("Say", "Intake On");
        }else if(gamepad1.dpad_down == true){
            robot.intake.setPosition(0);
            telemetry.addData("Say", "Intake Reverse");
        } else {
            robot.intake.setPosition(0.5);
            telemetry.addData("Say", "Intake Off");
        }
    }

    public void Navigate(){
        //find change in time since last execution
        time = runTime.time();
        dTime = time - lastTime;
        lastTime = time;
        telemetry.addData("dtime","%.2f", dTime);
    }
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
        Navigate();
        Drive();
        Score();
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
