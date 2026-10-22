package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * Flywheel PIDF velocity tuning OpMode. Runs from the Driver Station with gamepad1;
 * FTC Dashboard is optional (graphs and live gain tuning) and not needed to init.
 *
 * Triangle: toggle the flywheel between 0 and TARGET_TICKS_PER_SEC (fixed-speed tuning).
 * L3 (left stick press): toggle dynamic tuning, where the target sweeps smoothly
 * between DYNAMIC_MIN_TICKS_PER_SEC and DYNAMIC_MAX_TICKS_PER_SEC to mimic shooting from changing distances.
 *
 * Connect to FTC Dashboard at http://192.168.43.1:8080/dash. Tune kP/kI/kD/kF under
 * "Flywheel" and the targets under "FlywheelTuningOpMode"; plot targetTicksPerSec, ticksPerSec and
 * error in the Graph view. Tune kF first with kP/kI/kD at 0, then add kP, then kI/kD.
 */
@Config
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class FlywheelTuningOpMode extends LinearOpMode {

    public static double TARGET_TICKS_PER_SEC = 500;
    public static double DYNAMIC_MIN_TICKS_PER_SEC = 500;
    public static double DYNAMIC_MAX_TICKS_PER_SEC = 700;
    /** Seconds spent ramping in one full sweep min -> max -> min (not counting the holds). */
    public static double DYNAMIC_PERIOD_S = 6;
    /** Seconds to hold the target flat at the top and at the bottom of each sweep. */
    public static double DYNAMIC_HOLD_S = 2;
    /** Error (ticks/sec) that counts as "at speed" in telemetry. */
    public static double AT_SPEED_TOLERANCE_TICKS_PER_SEC = 25;

    public enum Mode {
        STOPPED,
        FIXED,
        DYNAMIC
    }

    @Override
    public void runOpMode() {
        // Setup dual telemetry (Driver Station + FTC Dashboard) only if Dashboard is running,
        // so the OpMode still inits from the Driver Station when Dashboard is off
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        Flywheel flywheel;
        try {
            flywheel = new Flywheel(hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addLine("No motor named \"flywheel\" in the active configuration.");
            telemetry.addLine("Add it on the Driver Station (Configure Robot), then re-init.");
            telemetry.update();
            waitForStart();
            return;
        }
        Mode mode = Mode.STOPPED;
        ElapsedTime dynamicTimer = new ElapsedTime();

        telemetry.addLine("Ready. Press START, then use gamepad1:");
        telemetry.addLine("Triangle = toggle fixed target, L3 = toggle dynamic sweep");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Triangle toggles fixed-speed tuning on/off
            if (gamepad1.triangleWasPressed()) {
                mode = (mode == Mode.FIXED) ? Mode.STOPPED : Mode.FIXED;
            }
            // L3 toggles dynamic tuning on/off; the sweep restarts at the min speed
            if (gamepad1.leftStickButtonWasPressed()) {
                mode = (mode == Mode.DYNAMIC) ? Mode.STOPPED : Mode.DYNAMIC;
                dynamicTimer.reset();
            }

            switch (mode) {
                case FIXED:
                    flywheel.setTargetTicksPerSec(TARGET_TICKS_PER_SEC);
                    break;
                case DYNAMIC:
                    // Cosine ramps with flat holds: up, hold at max, down, hold at min
                    double ramp = DYNAMIC_PERIOD_S / 2;
                    double hold = Math.max(0, DYNAMIC_HOLD_S);
                    double t = dynamicTimer.seconds() % (2 * ramp + 2 * hold);
                    double blend;
                    if (t < ramp) {
                        blend = (1 - Math.cos(Math.PI * t / ramp)) / 2;
                    } else if (t < ramp + hold) {
                        blend = 1;
                    } else if (t < 2 * ramp + hold) {
                        blend = (1 + Math.cos(Math.PI * (t - ramp - hold) / ramp)) / 2;
                    } else {
                        blend = 0;
                    }
                    flywheel.setTargetTicksPerSec(DYNAMIC_MIN_TICKS_PER_SEC + blend * (DYNAMIC_MAX_TICKS_PER_SEC - DYNAMIC_MIN_TICKS_PER_SEC));
                    break;
                case STOPPED:
                    flywheel.stop();
                    break;
            }
            flywheel.update();

            telemetry.addData("mode", mode);
            telemetry.addData("targetTicksPerSec", flywheel.getTargetTicksPerSec());
            telemetry.addData("ticksPerSec", flywheel.getTicksPerSec());
            telemetry.addData("rawTicksPerSec", flywheel.getRawTicksPerSec());
            telemetry.addData("error", flywheel.getError());
            telemetry.addData("power", flywheel.getPower());
            telemetry.addData("kP", Flywheel.kP);
            telemetry.addData("kI", Flywheel.kI);
            telemetry.addData("kD", Flywheel.kD);
            telemetry.addData("kF", Flywheel.kF);
            telemetry.addData("atSpeed", flywheel.atSpeed(AT_SPEED_TOLERANCE_TICKS_PER_SEC));
            telemetry.update();
        }
    }
}
