package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;

/**
 * Standard TeleOp template for Neuron Pathing.
 */
@TeleOp(name = "Neuron TeleOp", group = "Template")
public class NeuronTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Set your offsets (mm) and track width (inches)
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/0, /*yOffsetMm=*/0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/0);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1);
            drive.update();

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.update();
        }
    }
}
