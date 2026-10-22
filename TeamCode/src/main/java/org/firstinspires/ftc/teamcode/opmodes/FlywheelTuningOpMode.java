package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;

/**
 * Flywheel velocity tuning OpMode with FTC Dashboard integration.
 *
 * Triangle: toggle the flywheel between 0 and TARGET_RPM (fixed-speed tuning).
 * L3 (left stick press): toggle dynamic tuning, where the target sweeps smoothly
 * between DYNAMIC_MIN_RPM and DYNAMIC_MAX_RPM to mimic shooting from changing distances.
 *
 * Connect to FTC Dashboard at http://192.168.43.1:8080/dash. Tune the gains under
 * "Flywheel" and the targets under "FlywheelTuningOpMode"; plot targetRpm, rpm and
 * error in the Graph view.
 */
@Config
@TeleOp(name = "Flywheel Tuning (FTC Dashboard)", group = "Tuning")
public class FlywheelTuningOpMode extends LinearOpMode {

    public static double TARGET_RPM = 500;
    public static double DYNAMIC_MIN_RPM = 500;
    public static double DYNAMIC_MAX_RPM = 700;
    /** Seconds for one full sweep min -> max -> min. */
    public static double DYNAMIC_PERIOD_S = 6;
    /** Error (RPM) that counts as "at speed" in telemetry. */
    public static double AT_SPEED_TOLERANCE_RPM = 25;

    public enum Mode {
        STOPPED,
        FIXED,
        DYNAMIC
    }

    @Override
    public void runOpMode() {
        // Setup dual telemetry (Driver Station + FTC Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Flywheel flywheel = new Flywheel(hardwareMap);
        Mode mode = Mode.STOPPED;
        ElapsedTime dynamicTimer = new ElapsedTime();

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
                    flywheel.setTargetRpm(TARGET_RPM);
                    break;
                case DYNAMIC:
                    // Cosine sweep: smooth speed changes with no sudden jumps at the ends
                    double phase = 2 * Math.PI * dynamicTimer.seconds() / DYNAMIC_PERIOD_S;
                    double blend = (1 - Math.cos(phase)) / 2;
                    flywheel.setTargetRpm(DYNAMIC_MIN_RPM + blend * (DYNAMIC_MAX_RPM - DYNAMIC_MIN_RPM));
                    break;
                case STOPPED:
                    flywheel.stop();
                    break;
            }
            flywheel.update();

            telemetry.addData("mode", mode);
            telemetry.addData("targetRpm", flywheel.getTargetRpm());
            telemetry.addData("rpm", flywheel.getRpm());
            telemetry.addData("error", flywheel.getError());
            telemetry.addData("power", flywheel.getPower());
            telemetry.addData("atSpeed", flywheel.atSpeed(AT_SPEED_TOLERANCE_RPM));
            telemetry.update();
        }
    }
}
