/*
Copyright (c) 2018 FIRST

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.ShooterLab.TestOpModes;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Vector;
import java.util.concurrent.TimeUnit;


@TeleOp(name = "Velocity checker test", group = "Test")
public class VelocityCheckerTest extends LinearOpMode {
    static final double FGC_BALL_RADIUS = 0.063;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private DistanceSensor distantLaser;
    private DistanceSensor closeLaser;
    private DcMotor shooterMotor;
    private int power = 0;
    private double velocity = 0.0;

    @Override
    public void runOpMode() {
        distantLaser = hardwareMap.get(DistanceSensor.class, "distant_laser");
        closeLaser = hardwareMap.get(DistanceSensor.class, "close_laser");
        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");

        waitForStart();
        boolean upLastValue = false;
        boolean downLastValue = false;
        boolean ballInSight = false;

        double firstPointTime = 0;
        double secondPointTime = 0;

        while(opModeIsActive()) {
            // MOTOR CONTROL
            if (!upLastValue && gamepad1.dpad_up) {
                power += 1;
            }
            upLastValue = gamepad1.dpad_up;
            if (!downLastValue && gamepad1.dpad_down) {
                power -= 1;
            }
            downLastValue = gamepad1.dpad_down;
            power = (power + 11) % 11;
            shooterMotor.setPower((double)power / 10);


            double d = closeLaser.getDistance(DistanceUnit.MM);
            if (d < 55) {
                System.out.println("close:" + String.valueOf(d) + " " + runtime.time());
            }

            d = distantLaser.getDistance(DistanceUnit.MM);
            if (d < 70) {
                System.out.println("distant" + String.valueOf(d) + " " + runtime.time());
            }


//            if (distantLaser.getDistance(DistanceUnit.CM) < 6.9 && !ballInSight) {
//                firstPointTime = runtime.time();
//                ballInSight = true;
//            } else if (distantLaser.getDistance(DistanceUnit.CM) > 6.9 && ballInSight) {
//                secondPointTime = runtime.time();
//                ballInSight = false;
//            }
//            double timeWindow = (secondPointTime - firstPointTime) / 1000;
//            if (timeWindow > 0) {
//                velocity = FGC_BALL_RADIUS * 2 / timeWindow;
//            }

            telemetry.addData("power step", power);
            telemetry.addData("estimated velocity (m/s)", velocity);
//            telemetry.addData("detection time", timeWindow);
//            telemetry.addData("counter", counter);
            telemetry.addData("laser distance (mm)", distantLaser.getDistance(DistanceUnit.MM));
            // telemetry.addData()

            telemetry.update();
        }
    }
}
