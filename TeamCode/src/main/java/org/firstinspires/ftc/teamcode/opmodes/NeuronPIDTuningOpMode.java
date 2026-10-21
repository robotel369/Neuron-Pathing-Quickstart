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
 * Automated PID & Path Follower Tuning OpMode with FTC Dashboard integration.
 * 
 * Cycles the robot back and forth, or left and right, or turns in place.
 * Connect to FTC Dashboard at http://192.168.43.1:8080/dash to adjust PID values live.
 */
@Config
@TeleOp(name = "Neuron PID Tuning (FTC Dashboard)", group = "Tuning")
public class NeuronPIDTuningOpMode extends LinearOpMode {
    
    public static double DISTANCE = -25; // inches (negative target to counteract motor/localizer sign)
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
        // Setup dual telemetry (Driver Station + FTC Dashboard)
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/144.0, /*yOffsetMm=*/-144.0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/405.0 / 25.4);
        
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();

        boolean forward = true;
        Mode lastMode = null;
        
        while (opModeIsActive()) {
            boolean modeChanged = (tuningMode != lastMode);
            lastMode = tuningMode;

            if (modeChanged || !drive.isBusy()) {
                if (tuningMode == Mode.HOLD_FORWARD || tuningMode == Mode.HOLD_STRAFE || tuningMode == Mode.HOLD_TURN) {
                    forward = true;
                } else if (modeChanged) {
                    forward = true;
                } else {
                    forward = !forward;
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
            telemetry.update();
        }
    }
}
