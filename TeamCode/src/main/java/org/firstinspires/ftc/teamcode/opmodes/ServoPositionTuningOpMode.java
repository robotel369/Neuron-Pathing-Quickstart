package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;

/**
 * Finds servo positions for the shooter pivot and the gate. Runs with gamepad1.
 * Nudge each servo until it's where you want, read the number off telemetry, and put
 * it into ShooterPivot / Gate.
 *
 * Shooter pivot:
 *   Cross = BOTTOM, Square = TRANSITION, Triangle = SHOOT_FRONT, Circle = SHOOT_BACK
 *   Dpad up / down = nudge the pivot by STEP
 * Gate:
 *   R1 = toggle open/closed, Dpad right / left = nudge the gate by STEP
 */
@Config
@TeleOp(name = "Servo Position Tuning", group = "Tuning")
public class ServoPositionTuningOpMode extends LinearOpMode {

    public static double STEP = 0.01;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        ShooterPivot pivot = null;
        Gate gate = null;
        try {
            pivot = new ShooterPivot(hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addLine("No servos named \"pivotLeft\" / \"pivotRight\" in the configuration.");
        }
        try {
            gate = new Gate(hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addLine("No servo named \"gate\" in the configuration.");
        }
        telemetry.addLine("Pivot: cross/square/triangle/circle = presets, dpad up/down = nudge");
        telemetry.addLine("Gate: R1 = toggle, dpad right/left = nudge");
        telemetry.update();

        waitForStart();

        double gatePosition = gate != null ? Gate.CLOSED_POSITION : 0;

        while (opModeIsActive()) {
            if (pivot != null) {
                if (gamepad1.crossWasPressed()) pivot.bottom();
                if (gamepad1.squareWasPressed()) pivot.transition();
                if (gamepad1.triangleWasPressed()) pivot.shootFront();
                if (gamepad1.circleWasPressed()) pivot.shootBack();
                if (gamepad1.dpadUpWasPressed()) pivot.setPosition(pivot.getPosition() + STEP);
                if (gamepad1.dpadDownWasPressed()) pivot.setPosition(pivot.getPosition() - STEP);
                pivot.update();

                telemetry.addData("pivot pose", pivot.pose);
                telemetry.addData("pivot position", "%.3f", pivot.getPosition());
            }

            if (gate != null) {
                if (gamepad1.rightBumperWasPressed()) {
                    gate.toggle();
                    gatePosition = gate.state == Gate.State.OPEN ? Gate.OPEN_POSITION : Gate.CLOSED_POSITION;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    gatePosition = Math.min(1, gatePosition + STEP);
                    gate.setPosition(gatePosition);
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    gatePosition = Math.max(0, gatePosition - STEP);
                    gate.setPosition(gatePosition);
                }

                telemetry.addData("gate state", gate.state);
                telemetry.addData("gate position", "%.3f", gatePosition);
            }
            telemetry.update();
        }
    }
}
