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

    //variables for transloader control
    double loaderUp = 0.3;
    double loaderDown = 0.05;

    //shooter variables
    double sPower = 0.85;

    //variables for button handling
    boolean lastX = false;
    boolean lastY = false;
    boolean lastA = false;
    boolean lastB = false;

    public void Drive(){
        // Run wheels in not tank mode
        //set motor speeds
        if (gamepad1.right_bumper){
            driverMode = 0;
        } else if (gamepad1.left_bumper){
            driverMode = 2;
        } else {
            driverMode = 1;
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

    public void Score () {
        boolean shoot = false, runTrans = false, out = false, intakeOn = false;
        //get current joystick state to setup actions
        if (gamepad1.x != lastX && gamepad1.x) {
            //shoot
            shoot = !shoot;
        }
        if (gamepad1.a != lastA && gamepad1.a) {
            //transport on
            runTrans = !runTrans;
        }
        if (gamepad1.b != lastB && gamepad1.b) {
            //run in reverse
            out = !out;
        }
        if (gamepad1.y != lastY && gamepad1.y) {
            intakeOn = !intakeOn;
        }

        //code for running the transport
            if (runTrans) {
                telemetry.addData("Say", "Transporting");
                robot.transport.setPosition(0);
                if (runTime.time() % 2000 > 800) {
                    robot.transloader.setPosition(loaderDown);
                } else {
                    robot.transloader.setPosition(loaderUp);
                }
            } else if (out) {
                telemetry.addData("Say", "Outaking");
                robot.transport.setPosition(1);
                robot.transloader.setPosition(loaderDown);
            } else {
                robot.transloader.setPosition(loaderDown);
                robot.transport.setPosition(0.5);
            }

            //run the intake
            if (intakeOn) {
                robot.intake.setPosition(1);
                telemetry.addData("Say", "Intake On");
            } else if (out) {
                robot.intake.setPosition(0);
            } else {
                robot.intake.setPosition(0.5);
                telemetry.addData("Say", "Intake Off");
            }

            //code for running the shooter
            if (shoot) {
                telemetry.addData("Say", "Shooting");
                robot.rShooter.setPower(sPower);
                robot.lShooter.setPower(sPower);
                robot.loader.setPosition(0);
            } else {
                robot.lShooter.setPower(0);
                robot.rShooter.setPower(0);
                robot.loader.setPosition(0.5);
            }
        }

    public void Navigate() {
        //find change in time since last execution
        time = runTime.time();
        dTime = time - lastTime;
        lastTime = time;
        telemetry.addData("dtime", "%.2f", dTime);
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