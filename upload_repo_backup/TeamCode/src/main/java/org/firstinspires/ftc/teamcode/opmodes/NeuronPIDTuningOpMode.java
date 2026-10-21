package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Automated PID Tuning OpMode.
 * 
 * Cycles the robot back and forth, or left and right, or turns in place.
 * Includes "HOLD" modes to send the robot once and hold position.
 * Use FTC Dashboard (http://192.168.43.1:8080/dash) to adjust gains live.
 */
@Config
@TeleOp(name = "Neuron PID Tuning OpMode", group = "Tuning")
public class NeuronPIDTuningOpMode extends LinearOpMode {
    
    public static double DISTANCE = 25; // inches
    public static double ANGLE = 90;    // degrees
    
    public enum Mode {
        FORWARD_BACK,
        STRAFE_LEFT_RIGHT,
        TURN_IN_PLACE,
        HOLD_FORWARD,
        HOLD_STRAFE,
        HOLD_TURN
    }
    
    public static Mode tuningMode = Mode.FORWARD_BACK;
    
    @Override
    public void runOpMode() {
        // Setup dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        // Reset offsets and track width for new robot
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/0, /*yOffsetMm=*/0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/0);
        
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();

        boolean forward = false;
        Mode lastMode = null;
        
        while (opModeIsActive()) {
            // Check if user changed the mode in Dashboard
            boolean modeChanged = (tuningMode != lastMode);
            lastMode = tuningMode;

            if (modeChanged || !drive.isBusy()) {
                // Determine target direction
                if (tuningMode == Mode.HOLD_FORWARD || tuningMode == Mode.HOLD_STRAFE || tuningMode == Mode.HOLD_TURN) {
                    forward = true; // Always stay at the target in HOLD modes
                } else if (!modeChanged) {
                    forward = !forward; // Cycle back and forth in standard modes
                }

                double target = forward ? DISTANCE : 0;
                double angleTarget = forward ? Math.toRadians(ANGLE) : 0;

                switch (tuningMode) {
                    case FORWARD_BACK:
                    case HOLD_FORWARD:
                        drive.goToPoint(new Pose2D(target, 0, 0), false, 0.5);
                        break;
                    case STRAFE_LEFT_RIGHT:
                    case HOLD_STRAFE:
                        drive.goToPoint(new Pose2D(0, target, 0), false, 0.5);
                        break;
                    case TURN_IN_PLACE:
                    case HOLD_TURN:
                        drive.goToPoint(new Pose2D(0, 0, angleTarget), false, 0.5);
                        break;
                }
            }
            
            drive.update();
            
            telemetry.addData("Mode", tuningMode);
            telemetry.addData("Target", forward ? "Forward/Positive" : "Home/Zero");
            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("forwardError", drive.forwardError);
            telemetry.addData("strafeError", drive.strafeError);
            telemetry.addData("headingError", drive.hError);
            telemetry.addData("Drive State", drive.driveState);
            telemetry.addData("Is Busy", drive.isBusy());
            telemetry.addData("LF Pwr", drive.getLFPower());
            telemetry.update();
        }
    }
}
