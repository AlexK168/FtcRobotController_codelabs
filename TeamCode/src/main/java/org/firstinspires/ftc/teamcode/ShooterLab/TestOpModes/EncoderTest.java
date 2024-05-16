package org.firstinspires.ftc.teamcode.ShooterLab.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Encoder test", group="Test")
public class EncoderTest extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor motor   = null;
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_SPEED             = 0.6;

    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        motor  = hardwareMap.get(DcMotor.class, "motor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d",
                motor.getCurrentPosition()
        );
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_left) {
                encoderTurn(1, -45, 5.0);
            } else if (gamepad1.dpad_right) {
                encoderTurn(1, 45, 5.0);
            }
            double velocity = gamepad1.left_stick_y;
            motor.setPower(velocity);
        }
    }

    public void encoderTurn(double speed, double deg, double timeoutS) {
        int newTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = motor.getCurrentPosition() + (int)(deg / 360 * COUNTS_PER_MOTOR_REV);
            motor.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            motor.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) && motor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d", newTarget);
                telemetry.addData("Currently at",  " at %7d", motor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            motor.setPower(0);

            // Turn off RUN_TO_POSITION
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
