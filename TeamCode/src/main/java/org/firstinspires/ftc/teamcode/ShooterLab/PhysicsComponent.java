package org.firstinspires.ftc.teamcode.ShooterLab;

import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.g;
import static java.lang.Math.PI;

import org.la4j.Matrix;
import org.la4j.Vector;

public class PhysicsComponent {

    static public double inchToM(double value) {
        return 2.54 * value / 100;
    }

    static public Vector rotate(Vector vector, double angle) {
        return vector.multiply(Matrix.from2DArray(new double[][]{
                        {Math.cos(angle), Math.sin(angle)},
                        {Math.sin(-angle), Math.cos(angle)}
        }));
    }

    static public double degreeToRad(double angle) {
        return angle / 180 * PI;
    }

    static public double radToDegree(double angle) {
        return angle * 180 / PI;
    }

    static public Vector stocksForce(double radius, Vector velocity, double dynamicViscosity) {
        return velocity.multiply(6 * PI * dynamicViscosity * -1 * radius);
    }

    static public Vector gravityForce(double mass) {
        return g.multiply(mass);
    }

    static public Vector resistanceForce(
            double crossSection,
            Vector velocity,
            double airDensity,
            double resistanceCoefficient) {
        return velocity.multiply(-0.5 * velocity.norm() * crossSection * airDensity * resistanceCoefficient);
    }

    static public Vector magnusForce(
            double crossSection,
            Vector velocity,
            double rotationSpeed,
            double liftCoefficient,
            double airDensity) {
        return rotate(
                velocity.multiply(0.5 * liftCoefficient * airDensity * crossSection * rotationSpeed),
                PI / 2
        );
    }
}
