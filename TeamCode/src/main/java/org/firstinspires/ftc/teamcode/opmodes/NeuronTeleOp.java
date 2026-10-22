package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;


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
        Transfer transfer = new Transfer(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Shooter shooter = new Shooter(flywheel, transfer, gate);

        // Calibrate the Pinpoint during init, before the first drive.update() reads the pose
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();

            // Shoot macro on circle edge press: starts a shot, or cancels one that's running
            if (gamepad1.circleWasPressed()) {
                if (shooter.isBusy()) {
                    shooter.abort();
                } else {
                    shooter.start();
                }
            }
            shooter.update();

            // Intake power follows how far the triggers are pressed: R2 intakes, L2 outtakes
            // (the shoot macro doesn't use the intake). Small presses below 0.05 are ignored.
            double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
            intake.setPower(Math.abs(intakePower) > 0.05 ? intakePower : 0);

            // Manual gate/transfer controls are ignored while the macro runs, so they
            // can't close the gate or stop the transfer mid-shot
            if (!shooter.isBusy()) {
                // Gate toggle on R1 (right bumper) edge press: each press flips open/closed
                if (gamepad1.rightBumperWasPressed()) {
                    gate.toggle();
                }

                // Transfer toggle on dpad edge press: up starts/stops feeding, down starts/stops reversing
                if (gamepad1.dpadUpWasPressed()) {
                    if (transfer.state == Transfer.State.FEEDING) {
                        transfer.stop();
                    } else {
                        transfer.feed();
                    }
                } else if (gamepad1.dpadDownWasPressed()) {
                    if (transfer.state == Transfer.State.REVERSING) {
                        transfer.stop();
                    } else {
                        transfer.reverse();
                    }
                }
            }
            gate.update();
            intake.update();
            transfer.update();
            flywheel.update();

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("gate", gate.state);
            telemetry.addData("intake", intake.state);
            telemetry.addData("transfer", transfer.state);
            telemetry.addData("shooter", shooter.state);
            telemetry.addData("flywheel speed", flywheel.getTicksPerSec());
            telemetry.update();
        }
    }
}
