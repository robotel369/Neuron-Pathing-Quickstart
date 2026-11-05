package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Gate;

/**
 * Finds the gate servo's open and closed positions. Runs with gamepad1.
 * Nudge the servo until it's where you want, read the number off telemetry, and put it
 * into Gate.java. (The shooter pivot has its own "Shooter Pivot Tuning" OpMode.)
 *
 *   R1 = toggle open/closed (jumps to the saved positions)
 *   Dpad up / down = nudge the gate by STEP
 */
@Config
@TeleOp(name = "Gate Position Tuning", group = "Tuning")
public class GatePositionTuningOpMode extends LinearOpMode {

    public static double STEP = 0.01;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        Gate gate;
        try {
            gate = new Gate(hardwareMap);
        } catch (IllegalArgumentException e) {
            telemetry.addLine("No servo named \"gate\" in the configuration.");
            telemetry.update();
            waitForStart();
            return;
        }
        telemetry.addLine("Gate: R1 = toggle, dpad up/down = nudge");
        telemetry.update();

        waitForStart();

        double gatePosition = Gate.CLOSED_POSITION;

        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) {
                gate.toggle();
                gatePosition = gate.state == Gate.State.OPEN ? Gate.OPEN_POSITION : Gate.CLOSED_POSITION;
            }
            if (gamepad1.dpadUpWasPressed()) {
                gatePosition = Math.min(1, gatePosition + STEP);
                gate.setPosition(gatePosition);
            }
            if (gamepad1.dpadDownWasPressed()) {
                gatePosition = Math.max(0, gatePosition - STEP);
                gate.setPosition(gatePosition);
            }

            telemetry.addData("gate state", gate.state);
            telemetry.addData("gate position", "%.3f", gatePosition);
            telemetry.addData("saved", "OPEN %.3f / CLOSED %.3f", Gate.OPEN_POSITION, Gate.CLOSED_POSITION);
            telemetry.update();
        }
    }
}
