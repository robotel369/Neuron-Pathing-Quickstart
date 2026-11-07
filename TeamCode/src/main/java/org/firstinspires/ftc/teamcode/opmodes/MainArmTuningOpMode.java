package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Main arm PID tuning OpMode. Runs from the Driver Station with gamepad1; FTC Dashboard
 * is optional (graphs and live gain tuning). Start with the arm resting in its stowed
 * position, since the encoder zeroes at init.
 *
 * Triangle: move between POSITION_A_TICKS and POSITION_B_TICKS.
 * Left stick Y: nudge the goal up/down (MANUAL_TICKS_PER_SEC at full stick).
 * Cross: cut power (motor brakes) so you can check positions by hand. Any move re-enables.
 * Square: re-zero the encoder where the arm is now.
 *
 * Tuning order, in Dashboard under "MainArm":
 *  1. kG: with kP at 0, raise kG until the arm stays wherever you hold it near horizontal
 *     (set ANGLE_AT_ZERO_DEG first so the angle telemetry is right).
 *  2. kP: raise until the arm follows the setpoint closely without shaking.
 *  3. kD: add a little if it overshoots or bounces at the end of a move.
 *  4. kI: only if it settles short of the target; keep it small.
 * Graph goal, setpoint, position and error in the Dashboard Graph view.
 */
@Config
@TeleOp(name = "Main Arm Tuning", group = "Tuning")
public class MainArmTuningOpMode extends LinearOpMode {

    public static double POSITION_A_TICKS = 0;
    public static double POSITION_B_TICKS = 300;
    public static double MANUAL_TICKS_PER_SEC = 300;

    /** Shown on the Driver Station so you can tell which build is installed. */
    private static final String BUILD_TAG = "arm-tuning v3 (2026-11-06)";

    private String stage = "starting";

    @Override
    public void runOpMode() {
        try {
            runTuning();
        } catch (RuntimeException e) {
            if (isStopRequested()) throw e; // normal STOP, let the SDK handle it
            // Keep any error on screen until STOP instead of silently dropping back to init
            StackTraceElement[] trace = e.getStackTrace();
            while (!isStopRequested()) {
                telemetry.addData("Build", BUILD_TAG);
                telemetry.addLine("ERROR - the OpMode hit a problem:");
                telemetry.addData("During", stage);
                telemetry.addData("Error", e.toString());
                for (int i = 0; i < Math.min(4, trace.length); i++) {
                    telemetry.addLine("  at " + trace[i]);
                }
                telemetry.update();
                sleep(100);
            }
        }
    }

    private void runTuning() {
        stage = "setting up telemetry";
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        stage = "finding mainArm / backLeft";
        MainArm arm;
        try {
            arm = new MainArm(hardwareMap);
        } catch (IllegalArgumentException e) {
            // Keep the error on screen until STOP, so it can't flash past
            while (!isStopRequested()) {
                telemetry.addData("Build", BUILD_TAG);
                telemetry.addLine("CAN'T RUN: hardware missing from the active configuration.");
                telemetry.addLine("Needs a motor \"mainArm\" (arm power) and a motor \"backLeft\"");
                telemetry.addLine("(the arm's encoder is plugged into backLeft's port).");
                telemetry.addLine("Names are case-sensitive and must be set as motors.");
                telemetry.addData("SDK says", e.getMessage());
                telemetry.update();
                sleep(100);
            }
            return;
        }

        // Hold the shooter pivot at SHOOT_FRONT the whole time (moves at init).
        // Optional: if the pivot servos aren't in the config, arm tuning still runs.
        stage = "setting shooter pivot to SHOOT_FRONT";
        ShooterPivot pivot = null;
        try {
            pivot = new ShooterPivot(hardwareMap, /*moveToBottom=*/false);
            pivot.shootFront();
        } catch (IllegalArgumentException e) {
            pivot = null;
        }

        stage = "waiting for START";
        telemetry.addData("Build", BUILD_TAG);
        telemetry.addData("Shooter pivot", pivot == null
                ? "not in config (not held)" : "holding SHOOT_FRONT");
        telemetry.addLine("Hardware found. Arm must rest in its stowed spot (encoder zeroes now).");
        telemetry.addLine("Triangle = A/B, left stick = nudge, cross = power off, square = re-zero");
        telemetry.update();

        waitForStart();

        stage = "main loop";
        boolean atB = false;
        long lastNanos = System.nanoTime();

        while (opModeIsActive()) {
            long now = System.nanoTime();
            double dt = (now - lastNanos) / 1e9;
            lastNanos = now;

            if (gamepad1.triangleWasPressed()) {
                atB = !atB;
                arm.setTarget(atB ? POSITION_B_TICKS : POSITION_A_TICKS);
            }
            double stick = -gamepad1.left_stick_y;
            if (Math.abs(stick) > 0.05) {
                arm.adjustTarget(stick * MANUAL_TICKS_PER_SEC * dt);
            }
            if (gamepad1.crossWasPressed()) {
                arm.disable();
            }
            if (gamepad1.squareWasPressed()) {
                arm.resetEncoder();
            }
            arm.update();
            if (pivot != null) pivot.update();

            telemetry.addData("Build", BUILD_TAG);
            telemetry.addData("pivot", pivot == null ? "not in config"
                    : String.format("%s (%.3f)", pivot.pose, pivot.getPosition()));
            telemetry.addData("state", arm.state);
            telemetry.addData("goal", arm.getGoal());
            telemetry.addData("setpoint", arm.getSetpoint());
            telemetry.addData("position", arm.getPosition());
            telemetry.addData("error", arm.getError());
            telemetry.addData("power", arm.getPower());
            telemetry.addData("angle (deg)", arm.getAngleDeg());
            telemetry.addData("atTarget", arm.atTarget());
            telemetry.addData("kP / kI / kD", "%.5f / %.5f / %.5f", MainArm.kP, MainArm.kI, MainArm.kD);
            telemetry.addData("kG / kS", "%.3f / %.3f", MainArm.kG, MainArm.kS);
            telemetry.update();
        }
    }
}
