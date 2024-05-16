package org.firstinspires.ftc.teamcode.ShooterLab;

import org.la4j.Vector;

public class Constants {
    static final double FGC_BALL_RADIUS = 0.063;
    static final double FGC_BALL_MASS = 0.021;
    static final double AIR_DENSITY = 1.225;
    static final double RESISTANCE_COEF = 0.48;
    static final double DYNAMIC_VISCOSITY = 1.81 * 0.00001;
    static final double LIFT_COEF = 0.4;
    static final double ROTATION_SPEED = 4;
    static final double R_TARGET = FGC_BALL_RADIUS * 8;
    static final Vector g = Vector.fromArray(new double[]{0.0, -9.81});
    static final Vector SHOOTING_POINT_PIVOT_VECTOR = Vector.fromArray(new double[]{0.28, 0.03});
    static final Vector IMAGE_TO_TARGET_VECTOR = Vector.fromArray(new double[]{0, 0});
    static final double SHOOTING_WHEEL_RADIUS_CM = 4.5;

    static final String ANGLE_MOTOR_NAME = "angleMotor";
    static final String SHOOTER_MOTOR_NAME = "shooterMotor";
    static final String CAMERA_NAME = "Webcam 1";
    static final String POTENTIOMETER_NAME = "potentiometer";
    static final String LOWER_TOUCH_SENSOR_NAME = "lower_touch_sensor";
    static final String UPPER_TOUCH_SENSOR_NAME = "upper_touch_sensor";
}
