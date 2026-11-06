package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Manually finds the shooter pivot positions. The two pivot servos always move together
 * by the same amount (the right one is inverted in hardware): offset +0.2 moves both up by
 * 0.2 from their zero positions. Runs with gamepad1.
 *
 * Step 1, line the two sides up at offset 0 (Share goes there):
 *   Triangle / Cross = move ONLY the left servo's zero up / down
 *   Circle / Square  = move ONLY the right servo's zero up / down
 * Step 2, find each pose:
 *   Dpad up / down    = move both servos (offset + / - step)
 *   R1 / L1           = bigger / smaller step (0.001, 0.005, 0.01, 0.05)
 *   Dpad right / left = pick the next / previous preset (and move to its saved offset)
 *   Options           = save the current offset into the picked preset
 *   Share             = back to offset 0
 *
 * Saved values only last until the robot restarts: copy the numbers on telemetry into
 * ShooterPivot.java (LEFT_ZERO, RIGHT_ZERO and the *_OFFSET values).
 */
@TeleOp(name = "Shooter Pivot Tuning", group = "Tuning")
public class ShooterPivotTuningOpMode extends LinearOpMode {

    private static final double[] STEPS = {0.001, 0.005, 0.01, 0.05};
    private static final ShooterPivot.Pose[] PRESETS = {
            ShooterPivot.Pose.BOTTOM,
            ShooterPivot.Pose.TRANSITION,
            ShooterPivot.Pose.SHOOT_FRONT,
            ShooterPivot.Pose.SHOOT_BACK
    };

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        ShooterPivot pivot;
        try {
            pivot = new ShooterPivot(hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addLine("No servos named \"pivotLeft\" / \"pivotRight\" in the configuration.");
            telemetry.addLine("Add them on the Driver Station (Configure Robot), then re-init.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Shooter Pivot Tuning. Press START.");
        telemetry.addLine("Triangle/Cross = left zero, Circle/Square = right zero");
        telemetry.addLine("Dpad up/down = move both, R1/L1 = step size");
        telemetry.addLine("Dpad right/left = pick preset, Options = save, Share = offset 0");
        telemetry.update();

        waitForStart();

        int stepIndex = 2;   // 0.01
        int presetIndex = 0; // BOTTOM
        double offset = 0;
        String lastSaved = "";

        while (opModeIsActive()) {
            double step = STEPS[stepIndex];

            if (gamepad1.rightBumperWasPressed()) stepIndex = Math.min(STEPS.length - 1, stepIndex + 1);
            if (gamepad1.leftBumperWasPressed()) stepIndex = Math.max(0, stepIndex - 1);

            // Line the two sides up by moving one servo's zero at a time
            if (gamepad1.triangleWasPressed()) ShooterPivot.LEFT_ZERO = clip01(ShooterPivot.LEFT_ZERO + step);
            if (gamepad1.crossWasPressed()) ShooterPivot.LEFT_ZERO = clip01(ShooterPivot.LEFT_ZERO - step);
            if (gamepad1.circleWasPressed()) ShooterPivot.RIGHT_ZERO = clip01(ShooterPivot.RIGHT_ZERO + step);
            if (gamepad1.squareWasPressed()) ShooterPivot.RIGHT_ZERO = clip01(ShooterPivot.RIGHT_ZERO - step);

            // Move both servos together by the same amount
            if (gamepad1.dpadUpWasPressed()) offset += step;
            if (gamepad1.dpadDownWasPressed()) offset -= step;
            if (gamepad1.shareWasPressed()) offset = 0;

            // Pick a preset and jump to what's saved for it
            if (gamepad1.dpadRightWasPressed()) {
                presetIndex = (presetIndex + 1) % PRESETS.length;
                offset = pivot.presetOffset(PRESETS[presetIndex]);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                presetIndex = (presetIndex + PRESETS.length - 1) % PRESETS.length;
                offset = pivot.presetOffset(PRESETS[presetIndex]);
            }

            pivot.setOffset(offset);
            offset = pivot.getOffset(); // picks up the limit if it hit 0 or 1

            if (gamepad1.optionsWasPressed()) {
                save(PRESETS[presetIndex], offset);
                lastSaved = String.format("%s = %.3f", PRESETS[presetIndex], offset);
            }

            boolean atLimit = offset <= ShooterPivot.minOffset() || offset >= ShooterPivot.maxOffset();

            telemetry.addData("Offset", "%.3f%s", offset, atLimit ? "  (AT LIMIT: a servo is at 0 or 1)" : "");
            telemetry.addData("Left servo", "%.3f   (zero %.3f)", pivot.getLeftPosition(), ShooterPivot.LEFT_ZERO);
            telemetry.addData("Right servo", "%.3f   (zero %.3f)", pivot.getRightPosition(), ShooterPivot.RIGHT_ZERO);
            telemetry.addData("Step", "%.3f", step);
            telemetry.addData("Picked preset", PRESETS[presetIndex]);
            if (!lastSaved.isEmpty()) telemetry.addData("Last saved", lastSaved);
            telemetry.addLine();
            telemetry.addLine("Copy into ShooterPivot.java:");
            telemetry.addLine(String.format("LEFT_ZERO = %.3f;  RIGHT_ZERO = %.3f;",
                    ShooterPivot.LEFT_ZERO, ShooterPivot.RIGHT_ZERO));
            telemetry.addLine(String.format("BOTTOM_OFFSET = %.3f;", ShooterPivot.BOTTOM_OFFSET));
            telemetry.addLine(String.format("TRANSITION_OFFSET = %.3f;", ShooterPivot.TRANSITION_OFFSET));
            telemetry.addLine(String.format("SHOOT_FRONT_OFFSET = %.3f;", ShooterPivot.SHOOT_FRONT_OFFSET));
            telemetry.addLine(String.format("SHOOT_BACK_OFFSET = %.3f;", ShooterPivot.SHOOT_BACK_OFFSET));
            telemetry.update();
        }
    }

    private static void save(ShooterPivot.Pose pose, double offset) {
        switch (pose) {
            case BOTTOM:      ShooterPivot.BOTTOM_OFFSET = offset; break;
            case TRANSITION:  ShooterPivot.TRANSITION_OFFSET = offset; break;
            case SHOOT_FRONT: ShooterPivot.SHOOT_FRONT_OFFSET = offset; break;
            case SHOOT_BACK:  ShooterPivot.SHOOT_BACK_OFFSET = offset; break;
            default: break;
        }
    }

    private static double clip01(double v) {
        return Math.max(0, Math.min(1, v));
    }
}
