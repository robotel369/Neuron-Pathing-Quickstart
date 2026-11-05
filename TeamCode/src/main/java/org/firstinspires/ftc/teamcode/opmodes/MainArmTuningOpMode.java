package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MainArm;

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

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        MainArm arm;
        try {
            arm = new MainArm(hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addLine("Need motors \"mainArm\" and \"backLeft\" (arm encoder port) in the configuration.");
            telemetry.addLine("Add it on the Driver Station (Configure Robot), then re-init.");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addLine("Arm must rest in its stowed spot (encoder zeroes now).");
        telemetry.addLine("Triangle = A/B, left stick = nudge, cross = power off, square = re-zero");
        telemetry.update();

        waitForStart();

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
