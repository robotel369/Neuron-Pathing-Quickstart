package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Localizer sanity check / tuning aid.
 */
@TeleOp(name = "Neuron Localizer Test", group = "Tuning")
public class NeuronLocalizerTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Set your offsets (mm) here to test sign conventions
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/0, /*yOffsetMm=*/0);
        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();

        while (opModeIsActive()) {
            Pose2D pose = localizer.getPose();
            telemetry.addData("x (in)", pose.x);
            telemetry.addData("y (in)", pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.h));
            telemetry.update();
        }
    }
}
