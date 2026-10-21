package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Manual TeleOp on top of Neuron Pathing. Drive with the left stick's y/x and
 * rotate with the right stick's x -- robot-centric, no field-oriented math, so
 * it's a good first sanity check that your motors/directions are wired
 * correctly before layering in autonomous paths.
 *
 * Demonstrates the subsystem pattern using Gate (1 servo) and Intake (1 motor),
 * constructed alongside drive/localizer and updated every loop.
 */
@TeleOp(name = "Neuron TeleOp (Basic)")
public class NeuronTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/144.0, /*yOffsetMm=*/-144.0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/405.0 / 25.4);
        Gate gate = new Gate(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();

            // Gate toggle on 'A' button edge press
            if (gamepad1.aWasPressed()) {
                gate.toggle();
            }
            gate.update();

            // Intake control using triggers
            if (gamepad1.right_trigger > 0.1) {
                intake.intake();
            } else if (gamepad1.left_trigger > 0.1) {
                intake.outtake();
            } else {
                intake.stop();
            }
            intake.update();

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("gate", gate.state);
            telemetry.addData("intake", intake.state);
            telemetry.update();
        }
    }
}
