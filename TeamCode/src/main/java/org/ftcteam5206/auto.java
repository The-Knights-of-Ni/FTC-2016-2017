package org.ftcteam5206;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Parker on 2/13/17.
 */

@Autonomous(name="DriveBot_Auto", group="DriveBot")
public class auto extends LinearOpMode {
    //setup required variables
    public ElapsedTime runTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public hardwareDriveBot robot = new hardwareDriveBot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        driveForward(25,800);
        Shoot();
        driveForward(50,1000);
    }

    //drives motors for a certain time
    public void driveForward(int power, int stopTime){
        double initialTime = runTime.time();
        double timer  = 0;

        while(timer<stopTime){
            robot.lfMotor.setPower(power);
            robot.rfMotor.setPower(power);
            robot.rbMotor.setPower(power);
            robot.lbMotor.setPower(power);

            timer = runTime.time() - initialTime;

            telemetry.addData("DriveTime", "%.2f", timer/1000);
        }
        robot.lfMotor.setPower(0);
        robot.rfMotor.setPower(0);
        robot.rbMotor.setPower(0);
        robot.lbMotor.setPower(0);
    }

    //try to shoot a particle
    public void Shoot(){
        double initialTime = runTime.time();
        double timer  = 0;
        int allotedTime = 10000;
        double sPower = 0.75;

        //variables for tranloader control
        double loaderUp = 0.3;
        double loaderDown = 0.05;

        while(timer < allotedTime){
            robot.rShooter.setPower(sPower);
            robot.lShooter.setPower(sPower);
            robot.loader.setPosition(0);
            robot.transport.setPosition(0);

            if (runTime.time() % 2000 > 800) {
                robot.transloader.setPosition(loaderDown);
            } else {
                robot.transloader.setPosition(loaderUp);
            }

            timer = runTime.time() - initialTime;

            telemetry.addData("ShootTime", "%.2f", timer/1000);
        }

        robot.rShooter.setPower(0);
        robot.lShooter.setPower(0);
        robot.loader.setPosition(0.5);
        robot.transport.setPosition(0.5);

    }
}

