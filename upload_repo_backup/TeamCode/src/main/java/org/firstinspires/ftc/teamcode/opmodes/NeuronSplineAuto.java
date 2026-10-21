package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Standard Autonomous template for Neuron Pathing.
 */
@Autonomous(name = "Neuron Spline Auto", group = "Template")
public class NeuronSplineAuto extends LinearOpMode {
    NeuronLocalizer localizer;
    NeuronDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Set your offsets (mm) and track width (inches)
        localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/0, /*yOffsetMm=*/0);
        drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/0);

        localizer.setPosition(new Pose2D(0, 0, 0));
        localizer.calibrate(true);

        waitForStart();
        if (isStopRequested()) return;

        // PASTE PATH PLANNER CODE BELOW
        
        // PASTE PATH PLANNER CODE ABOVE

        telemetry.addData("Status", "Path Finished");
        telemetry.update();
    }

    private void driveToTarget() {
        while (opModeIsActive() && drive.isBusy()) {
            drive.update();
            telemetry.addData("state", drive.driveState);
            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.update();
        }
    }

    private void waitMs(double ms) {
        double start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < ms) {
            drive.update();
            idle();
        }
    }
}
