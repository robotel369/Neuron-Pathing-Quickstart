package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Example autonomous showing the exact shape of code the Neuron Path Planner
 * (tools/neuron-path-planner.html) generates: set a starting pose, then chain
 * splineToPoint()/goToPoint() calls, each followed by driveToTarget().
 *
 * Shows how subsystems (Gate, Intake) drop into the autonomous sequence alongside NeuronDrive.
 */
@Autonomous(name = "Neuron Spline Auto (Example)")
public class NeuronSplineAuto extends LinearOpMode {
    NeuronLocalizer localizer;
    NeuronDrive drive;
    Gate gate;
    Intake intake;

    @Override
    public void runOpMode() throws InterruptedException {
        localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/144.0, /*yOffsetMm=*/-144.0);
        drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/405.0 / 25.4);
        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);

        gate.close();
        intake.stop();

        // Set this to wherever your robot actually starts on the field.
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();
        if (isStopRequested()) return;

        // Drive to first waypoint
        drive.splineToPoint(new Pose2D(24, 24, Math.toRadians(90)), false, false, 1, 0.8);
        driveToTarget();

        // Run intake
        intake.intake();
        waitMs(500);

        // Drive to second waypoint
        drive.goToPoint(new Pose2D(24, 0, 0), false, 0.6);
        driveToTarget();

        // Open gate & stop intake
        gate.open();
        waitMs(300);
        intake.stop();
    }

    // Blocks until the active goToPoint()/splineToPoint() move finishes. NeuronDrive
    // only moves while update() is being called, so every move needs one of these.
    private void driveToTarget() {
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            gate.update();
            intake.update();
            telemetry.addData("state", drive.driveState);
            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("gate", gate.state);
            telemetry.addData("intake", intake.state);
            telemetry.update();
        }
    }

    // Pauses for ms milliseconds while still actively holding position and
    // updating subsystems.
    private void waitMs(double ms) {
        double start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            drive.update();
            gate.update();
            intake.update();
            idle();
        }
    }
}
