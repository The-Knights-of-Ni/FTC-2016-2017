package org.ftcteam5206.teleop;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Dev on 12/1/2016.
 */
@TeleOp(name="TeleopTest", group="DriveBot")
public class PlaceholderTeleop extends OpMode{

        /* Declare OpMode members. */
        hardwareDriveBot robot = new hardwareDriveBot();

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
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
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            double left;
            double right;

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.lMotor.setPower(left);
            robot.rMotor.setPower(right);

            // Send telemetry message to signify robot running;
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
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
