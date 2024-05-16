package org.firstinspires.ftc.teamcode.ShooterLab;

import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.ANGLE_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.CAMERA_NAME;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.IMAGE_TO_TARGET_VECTOR;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.LOWER_TOUCH_SENSOR_NAME;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.POTENTIOMETER_NAME;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.SHOOTER_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.SHOOTING_POINT_PIVOT_VECTOR;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.UPPER_TOUCH_SENSOR_NAME;
import static org.firstinspires.ftc.teamcode.ShooterLab.PhysicsComponent.inchToM;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.la4j.Vector;

import java.util.List;

@TeleOp(name="Shooter lab")

public class ShooterLab extends LinearOpMode {
    // Hardware
    private DcMotor angleMotor = null;
    private DcMotor shooterMotor = null;
    private TouchSensor lowerTouchSensor = null;
    private TouchSensor upperTouchSensor = null;
    private ElapsedTime runtime = new ElapsedTime();
    private CameraName camera = null;
    private AnalogInput potentiometer = null;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private double lowerPotentiometerValue = 0;
    private double upperPotentiometerValue = 0;


    private void initHardware() {
        angleMotor  = hardwareMap.get(DcMotor.class, ANGLE_MOTOR_NAME);
        shooterMotor = hardwareMap.get(DcMotor.class, SHOOTER_MOTOR_NAME);
        potentiometer = hardwareMap.get(AnalogInput.class, POTENTIOMETER_NAME);
        lowerTouchSensor = hardwareMap.get(TouchSensor.class, LOWER_TOUCH_SENSOR_NAME);
        upperTouchSensor = hardwareMap.get(TouchSensor.class, UPPER_TOUCH_SENSOR_NAME);
    }

    private void initAprilTag() {
        camera = hardwareMap.get(WebcamName.class, CAMERA_NAME);
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(camera);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    private AprilTagPoseFtc getTargetPosition() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                return detection.ftcPose;
            }
        }
        return null;
    }

    private void calibratePotentiometer() {
        while (!upperTouchSensor.isPressed()) {
            angleMotor.setPower(-0.2);
        }
        angleMotor.setPower(0);
        upperPotentiometerValue = potentiometer.getVoltage();
        while (!lowerTouchSensor.isPressed()) {
            angleMotor.setPower(0.1);
        }
        angleMotor.setPower(0);
        lowerPotentiometerValue = potentiometer.getVoltage();
    }

    private void setLauncherAngle(double angle) {
        final double desiredValue = (angle / 90) * (upperPotentiometerValue - lowerPotentiometerValue) + lowerPotentiometerValue;
        while (desiredValue > potentiometer.getVoltage()
                && !lowerTouchSensor.isPressed()) {
            angleMotor.setPower(0.1);
        }
        angleMotor.setPower(0);
        while (desiredValue < potentiometer.getVoltage()
                && !upperTouchSensor.isPressed()) {
            angleMotor.setPower(-0.15);
        }
        angleMotor.setPower(0);
    }

    private Vector searchForTarget() {
        // lower to the lowest position
        angleMotor.setPower(0.1);
        while (!lowerTouchSensor.isPressed()) {}
        angleMotor.setPower(0);

        for (int i = 1; i < 90; i++) {
            setLauncherAngle(i);
            final AprilTagPoseFtc position = getTargetPosition();
            if (position == null) {
                continue;
            }
            double cameraToImageDistance = inchToM(Math.sqrt(
                    Math.pow(position.x,2) +
                    Math.pow(position.y,2) +
                    Math.pow(position.z,2)
            ));
            System.out.println("camera to image dist: " + cameraToImageDistance);
            double beta =  position.elevation;
            System.out.println("alpha: " + (double)i);
            System.out.println("beta: " + beta);
            final double angle = PhysicsComponent.degreeToRad((double)i + beta);
            double dist = Math.cos(angle) * cameraToImageDistance;
            double height = Math.sin(angle) * cameraToImageDistance;
            System.out.println("dist: " + dist);
            System.out.println("height: " + height);
            final Vector pivot = PhysicsComponent.rotate(
                    SHOOTING_POINT_PIVOT_VECTOR,
                    PhysicsComponent.degreeToRad((double)i)
            );
            System.out.println("pivot0: " + pivot.get(0));
            System.out.println("pivot1: " + pivot.get(1));
            return Vector
                    .fromArray(new double[]{dist, height})
                    .add(pivot)
                    .add(IMAGE_TO_TARGET_VECTOR);
        }
        return null;
    }

    private void manual() {
        while (true) {
            angleMotor.setPower(0.2 * gamepad1.left_stick_y);
            if (gamepad1.right_bumper) {
                shooterMotor.setPower(0.8);
            } else {
                shooterMotor.setPower(0);
            }
        }
    }

    @Override
    public void runOpMode() {

        // init motors & sensors
        initHardware();
        initAprilTag();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // manual();

        // calibrate potentiometer
        calibratePotentiometer();

        // create solution class
        Solution solution = new Solution();

        while (opModeIsActive()) {
            // wait for player to start
            while (!gamepad1.right_bumper) {}

            // search for a target
            // retrieved vector represents coordinates of a target
            // related to an axis of the launcher rotation
            Vector target = searchForTarget();
            if (target == null) {
                telemetry.addLine("Target not found");
                telemetry.update();
                return;
            }

            telemetry.addLine("Target position: " +
                    String.valueOf(target.get(0)) + " " +
                    String.valueOf(target.get(1))
            );
            telemetry.update();

            while (!gamepad1.right_bumper) {}

            final Integer angle = solution.solve(target.get(0), target.get(1));
            if (angle == null) {
                telemetry.addLine("Solution not found");
                telemetry.update();
                return;
            }
            telemetry.addLine("Solution: " + angle);
            telemetry.update();
            // set angle
            setLauncherAngle(angle);

            while (!gamepad1.right_bumper) {
            }
            // shoot
            shooterMotor.setPower(1);

            // wait for player to stop
            while (!gamepad1.square) {
            }
            shooterMotor.setPower(0);
        }
    }
}
