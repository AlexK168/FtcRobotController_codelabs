package org.firstinspires.ftc.teamcode.ShooterLab;

import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.FGC_BALL_MASS;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.FGC_BALL_RADIUS;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.ROTATION_SPEED;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.R_TARGET;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.SHOOTING_POINT_PIVOT_VECTOR;
import static org.firstinspires.ftc.teamcode.ShooterLab.Constants.SHOOTING_WHEEL_RADIUS_CM;

import org.javatuples.Pair;
import org.la4j.Vector;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Solution {

    private double errorFunction(Vector currentPosition, Vector desirablePosition) {
        return desirablePosition.subtract(currentPosition).length();
    }

    public Integer solve(double dist, double height) {
        final double step = 0.01;
        final Projectile projectile = new Projectile(
                FGC_BALL_RADIUS,
                ROTATION_SPEED,
                FGC_BALL_MASS
        );

        final Vector target = Vector.fromArray(new double[]{dist, height});
        List<Pair<Integer, Double>> solutions = new ArrayList<>();
        for (int i = 89; i >= 0; i--) {
            final double i_rad = PhysicsComponent.degreeToRad(i);
            final Vector startPosition = PhysicsComponent.rotate(
              SHOOTING_POINT_PIVOT_VECTOR, i_rad
            );
            // 4.5 - radius of a wheel.
            // Start velocity derived theoretically: 4.5 * PI m/s = 14.13 m/s
            // Camera showed approximately: 14-15 m/s
            final double startVelocityValue = SHOOTING_WHEEL_RADIUS_CM * Math.PI;
            final Vector startVelocity = Vector.fromArray(new double[]{
                    startVelocityValue * Math.cos(i_rad),
                    startVelocityValue * Math.sin(i_rad)
            });

            projectile.reset(startPosition, startVelocity);

            List<Double> errors = new ArrayList<>();
            while (true) {
                final double error = errorFunction(projectile.currentPosition, target);
                if (error <= R_TARGET && projectile.velocity.get(1) < 0) {
                    errors.add(error);
                } else if (error > R_TARGET && projectile.currentPosition.get(0) > target.get(0)) {
                    // overshoot
                    break;
                }
                projectile.update(step);
                if (projectile.currentPosition.get(1) < startPosition.get(1)) {
                    break;
                }
            }
            if (errors.size() > 0) {
                final Pair<Integer, Double> pair =
                        new Pair<>(i, Collections.min(errors));
                solutions.add(pair);
            }
        }
        if (solutions.isEmpty()) {
            return null;
        }
        return solutions.get(0).getValue0();
    }
}
