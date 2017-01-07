package org.ftcteam5206.teleop;

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

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.ftcteam5206.RobotName;
import org.ftcteam5206.utils.Button.ButtonHandler;


//copied template from PushbotTeleopTank_Interative.java

@TeleOp(name="Dri3veBot_test", group="DriveBot")
public class Teleop1 extends LinearOpMode{

    RobotName robot = new RobotName();
    ButtonHandler pad1; //TODO: Totally not right.
    public enum robotState
    {
        STOPPED, DRIVING, INTAKE, LAUNCHER
    }

    private robotState currentState = robotState.STOPPED;


    public void updatePosition(RobotName robot, Gamepad gamepad1, Gamepad gamepad2)
    {
        double left  = -gamepad1.left_stick_y + gamepad1.left_stick_x;
        double right = -gamepad1.left_stick_y - gamepad1.left_stick_x;
        double max;

        max = Math.max(Math.abs(left), Math.abs(right));
        if (max > 1.0)
        {
            left /= max;
            right /= max;
        }

        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);

        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        updateTelemetry(telemetry);

    }

    public void updateIntakeForward(RobotName robot, ButtonHandler gamePad1, ButtonHandler gamePad2)
    {
        if (gamePad1.toggle(pad1.buttons.LEFT_STICK_BUTTON))
        {
            robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            robot.intake.setPower(20);
        }
        else if (!gamePad1.toggle(pad1.buttons.LEFT_STICK_BUTTON))
        {
            robot.intake.setPower(0);
        }
    }

    public void updateIntakeReverse(RobotName robot, ButtonHandler gamePad1, ButtonHandler gamePad2)
    {
        if (gamePad1.toggle(pad1.buttons.LEFT_BUMPER))
        {
            robot.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            robot.intake.setPower(20);
        }
        else if (!gamePad1.toggle(pad1.buttons.LEFT_BUMPER))
        {
            robot.intake.setPower(0);
        }
    }

    public void updateTransport(RobotName robot, ButtonHandler gamePad1, ButtonHandler gamePad2)
    {
        if (gamePad2.toggle(pad1.buttons.LEFT_STICK_BUTTON))
        {
            robot.transportIn.setDirection(Servo.Direction.FORWARD);
            robot.leftTransport.setDirection(Servo.Direction.FORWARD);
            robot.rightTransport.setDirection(Servo.Direction.REVERSE);
            robot.transportOut.setDirection(Servo.Direction.FORWARD);
        }

        //SOMEONE SHOULD ADD THE PROPER CODE HERE
        //else if (gamePad2.toggle(pad1.buttons.LEFT_STICK_BUTTON))
    }

    public void updateForks(RobotName robot, ButtonHandler gamePad1, ButtonHandler gamePad2)
    {
        if (gamePad1.toggle(pad1.buttons.RIGHT_STICK_BUTTON))
        {
            robot.forkRelease.setDirection(Servo.Direction.FORWARD);
        }
        else if (!gamePad1.toggle(pad1.buttons.RIGHT_STICK_BUTTON))
        {
            robot.forkRelease.setPosition(0.5);
        }
    }

    public void updateClasp(RobotName robot, Gamepad pad1, Gamepad pad2)
    {
        double claspPos = pad1.left_stick_y;

        robot.clasp.setPosition(claspPos);
    }

    public void updateCappRelease (RobotName robot, ButtonHandler gamePad1, ButtonHandler gamePad2)
    {
        if (gamePad1.toggle(pad1.buttons.RIGHT_BUMPER))
        {
            robot.capRelease.setDirection(Servo.Direction.FORWARD);
        }
        else if (!gamePad1.toggle(pad1.buttons.RIGHT_BUMPER))
        {
            robot.capRelease.setPosition(0.5);
        }
    }

    public void updateBeaconPusher (RobotName robot, ButtonHandler gamePad1, ButtonHandler gamePad2){
        if (gamePad2.toggle(pad1.buttons.Y)){
            if (robot.beaconPusher.getPosition() < 0.5){
                robot.beaconPusher.setPosition(0.9);
            }
            else {
                robot.beaconPusher.setPosition(0.1);
            }

        }
    }



    @Override
    public void runOpMode() throws InterruptedException
    {

        ButtonHandler pad1 = new ButtonHandler();
        ButtonHandler pad2 = new ButtonHandler();
        robotState nextState = robotState.STOPPED;

        robot.init(hardwareMap);
        waitForStart();

        currentState = robotState.DRIVING;

        while (opModeIsActive())
        {
            try {
                pad1.updateButtons(gamepad1.toByteArray(), gamepad1.left_trigger, gamepad1.right_trigger);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }
            try {
                pad2.updateButtons(gamepad2.toByteArray(), gamepad2.left_trigger, gamepad2.right_trigger);
            } catch (RobotCoreException e) {
                e.printStackTrace();
            }

            switch (currentState)
            {
                case DRIVING:
                    updatePosition(robot, gamepad1, gamepad2);
                    updateIntakeForward(robot, pad1, pad2);
                    updateIntakeReverse(robot, pad1, pad2);
                    updateTransport(robot, pad1, pad2);
                    updateForks(robot, pad1, pad2);
                    updateCappRelease(robot, pad1, pad2);
                    updateClasp(robot, gamepad1, gamepad2);
                    updateBeaconPusher(robot, pad1, pad2);
                    nextState = robotState.DRIVING;
                    break;
                case INTAKE:
                    break;
                case LAUNCHER:
                    break;

            }
            if (currentState != nextState)
            {
                //TODO print message
                currentState = nextState;
            }
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */

}
