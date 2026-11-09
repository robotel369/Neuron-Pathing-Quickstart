package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Manually finds the shooter pivot positions. Both pivot servos always get the same
 * position (the right one is inverted in hardware), using the full range:
 * 1 = BOTTOM (ground), 0 = SHOOT_BACK. Runs with gamepad1.
 *
 * The servos start RELEASED (limp, not holding anything) and only hold a position once
 * one is commanded.
 *   Triangle          = go to the next pose and hold it. The first press goes to BOTTOM;
 *                       then TRANSITION -> SHOOT_FRONT -> SHOOT_BACK -> BOTTOM
 *   Cross             = go to the previous pose and hold it
 *   Circle            = release: both servos let go and can be moved by hand
 *   Dpad up / down    = fine-adjust the held position (+ / - step); ignored while released
 *   R1 / L1           = bigger / smaller step (0.001, 0.005, 0.01, 0.05)
 *   Options           = save the held position into the current pose
 *   Share             = go to BOTTOM and hold it
 *
 * The servos don't report where they are, so while released there's no position number
 * to read. Use it to look at the mechanism, then hold a pose and fine-adjust to match.
 * Saved values only last until the robot restarts: copy the numbers on telemetry into
 * ShooterPivot.java (the *_POSITION values).
 */
@TeleOp(name = "Shooter Pivot Tuning", group = "Tuning")
public class ShooterPivotTuningOpMode extends LinearOpMode {

    private static final double[] STEPS = {0.001, 0.005, 0.01, 0.05};

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        ShooterPivot pivot;
        try {
            pivot = new ShooterPivot(hardwareMap, /*moveToBottom=*/false); // starts limp
        } catch (IllegalArgumentException e) {
            telemetry.addLine("No servos named \"pivotLeft\" / \"pivotRight\" in the configuration.");
            telemetry.addLine("Add them on the Driver Station (Configure Robot), then re-init.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Shooter Pivot Tuning. Servos are RELEASED (limp). Press START.");
        telemetry.addLine("Triangle = next pose, Cross = previous, Circle = release");
        telemetry.addLine("Dpad up/down = fine adjust, R1/L1 = step size");
        telemetry.addLine("Options = save to this pose, Share = BOTTOM");
        telemetry.update();

        waitForStart();

        ShooterPivot.Pose[] presets = ShooterPivot.PRESETS;
        int stepIndex = 2;    // 0.01
        int presetIndex = -1; // nothing commanded yet
        double position = 0;
        String lastSaved = "";

        while (opModeIsActive()) {
            double step = STEPS[stepIndex];

            if (gamepad1.rightBumperWasPressed()) stepIndex = Math.min(STEPS.length - 1, stepIndex + 1);
            if (gamepad1.leftBumperWasPressed()) stepIndex = Math.max(0, stepIndex - 1);

            // Commands that move the servos and make them hold
            if (gamepad1.triangleWasPressed()) {
                // The very first press goes to BOTTOM
                presetIndex = (presetIndex < 0) ? 0 : (presetIndex + 1) % presets.length;
                position = pivot.presetPosition(presets[presetIndex]);
                pivot.setPosition(position);
            }
            if (gamepad1.crossWasPressed()) {
                presetIndex = (presetIndex <= 0) ? presets.length - 1 : presetIndex - 1;
                position = pivot.presetPosition(presets[presetIndex]);
                pivot.setPosition(position);
            }
            if (gamepad1.shareWasPressed()) {
                presetIndex = 0;
                position = pivot.presetPosition(presets[presetIndex]);
                pivot.setPosition(position);
            }

            // Fine adjust only while holding
            if (!pivot.isReleased()) {
                if (gamepad1.dpadUpWasPressed()) {
                    pivot.setPosition(position + step);
                    position = pivot.getTarget();
                }
                if (gamepad1.dpadDownWasPressed()) {
                    pivot.setPosition(position - step);
                    position = pivot.getTarget();
                }
            }

            // Let go
            if (gamepad1.circleWasPressed()) {
                pivot.release();
            }
            // Walks the servos toward the target at ShooterPivot.MAX_SPEED_PER_SEC
            pivot.update();

            if (gamepad1.optionsWasPressed() && !pivot.isReleased() && presetIndex >= 0) {
                ShooterPivot.savePreset(presets[presetIndex], position);
                lastSaved = String.format("%s = %.3f", presets[presetIndex], position);
            }

            if (pivot.isReleased()) {
                telemetry.addData("State", "RELEASED (limp) - Triangle/Cross/Share to hold a pose");
                if (presetIndex >= 0) {
                    telemetry.addData("Last pose", "%s (%.3f)", presets[presetIndex], position);
                }
            } else {
                boolean onSaved = Math.abs(position - pivot.presetPosition(presets[presetIndex])) < 1e-9;
                telemetry.addData("State", pivot.isMoving()
                        ? String.format("MOVING (now %.3f)", pivot.getPosition()) : "HOLDING");
                telemetry.addData("Pose", "%s%s", presets[presetIndex], onSaved ? "" : "  (adjusted, not saved)");
                telemetry.addData("Position (both servos)", "%.3f", position);
            }
            telemetry.addData("Step", "%.3f", step);
            if (!lastSaved.isEmpty()) telemetry.addData("Last saved", lastSaved);
            telemetry.addLine();
            telemetry.addLine("Copy into ShooterPivot.java:");
            telemetry.addLine(String.format("BOTTOM_POSITION = %.3f;", ShooterPivot.BOTTOM_POSITION));
            telemetry.addLine(String.format("TRANSITION_POSITION = %.3f;", ShooterPivot.TRANSITION_POSITION));
            telemetry.addLine(String.format("SHOOT_FRONT_POSITION = %.3f;", ShooterPivot.SHOOT_FRONT_POSITION));
            telemetry.addLine(String.format("SHOOT_BACK_POSITION = %.3f;", ShooterPivot.SHOOT_BACK_POSITION));
            telemetry.update();
        }
    }
}
