package org.firstinspires.ftc.teamcode.ShooterLab;

import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.AIR_DENSITY;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.DYNAMIC_VISCOSITY;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.LIFT_COEF;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.RESISTANCE_COEF;

import static java.lang.Math.PI;

import org.la4j.Vector;

public class Projectile {
    private final double radius;
    public Vector currentPosition;
    public Vector velocity;
    private final double rotationSpeed;
    private final double mass;
    private final double crossSection;

    public Projectile(
            double radius,
            double rotationSpeed,
            double mass
    ) {
        this.radius = radius;
        this.currentPosition = Vector.fromArray(new double[]{0, 0});
        this.velocity = Vector.fromArray(new double[]{0, 0});
        this.rotationSpeed = rotationSpeed;
        this.mass = mass;
        this.crossSection = PI * radius * radius;
    }

    private void updatePosition(double delta) {
        currentPosition = currentPosition.add(velocity.multiply(delta));
    }

    private void updateVelocity(double delta) {
        final Vector gravityForce = PhysicsComponent.gravityForce(mass);

        final Vector magnusForce = PhysicsComponent.magnusForce(
                crossSection,
                velocity,
                rotationSpeed,
                LIFT_COEF,
                AIR_DENSITY
        );

        final Vector resistanceForce = PhysicsComponent.resistanceForce(
                crossSection,
                velocity,
                AIR_DENSITY,
                RESISTANCE_COEF
        );

        final Vector stocksForce = PhysicsComponent.stocksForce(
                radius,
                velocity,
                DYNAMIC_VISCOSITY
        );

        final Vector increment = Vector.zero(2)
                .add(gravityForce)
                .add(magnusForce)
                .add(stocksForce)
                .add(resistanceForce)
                .multiply(delta / mass);

        velocity = velocity.add(increment);
    }

    public void update(double delta) {
        updateVelocity(delta);
        updatePosition(delta);
    }

    public void reset(Vector startPosition, Vector startVelocity) {
        velocity = startVelocity;
        currentPosition = startPosition;
    }
}
