package org.firstinspires.ftc.teamcode.ShooterLab;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class LauncherController {
    final private DcMotor angleMotor;
    final private DcMotor shooterMotor;
    private final TouchSensor lowerTouchSensor;
    private final TouchSensor upperTouchSensor;
    private final AnalogInput potentiometer;
    private Double upperPotentiometerValue = null;
    private Double lowerPotentiometerValue = null;


    public LauncherController(DcMotor angleMotor, DcMotor shooterMotor, TouchSensor lowerTouchSensor, TouchSensor upperTouchSensor, AnalogInput potentiometer) {
        this.angleMotor = angleMotor;
        this.shooterMotor = shooterMotor;
        this.lowerTouchSensor = lowerTouchSensor;
        this.upperTouchSensor = upperTouchSensor;
        this.potentiometer = potentiometer;
    }

    private void calibrate() {
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

    public void setLauncherAngle(double angle, AngleUnit unit) {
        final double desiredValue = (unit == AngleUnit.RADIANS)
                ? (angle / Math.PI * 2) : (angle / 90)
                * (upperPotentiometerValue - lowerPotentiometerValue) + lowerPotentiometerValue;
        while (desiredValue > potentiometer.getVoltage() && !lowerTouchSensor.isPressed()) {
            angleMotor.setPower(0.1);
        }
        angleMotor.setPower(0);
        while (desiredValue < potentiometer.getVoltage() && !upperTouchSensor.isPressed()) {
            angleMotor.setPower(-0.15);
        }
        angleMotor.setPower(0);
    }

    public void toLowerPosition() {
        angleMotor.setPower(0.1);
        while (!lowerTouchSensor.isPressed()) {}
        angleMotor.setPower(0);
    }

    public void toUpperPosition() {
        angleMotor.setPower(-0.15);
        while (!lowerTouchSensor.isPressed()) {}
        angleMotor.setPower(0);
    }
}
