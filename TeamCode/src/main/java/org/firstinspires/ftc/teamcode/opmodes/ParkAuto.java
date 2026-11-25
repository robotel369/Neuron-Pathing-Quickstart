package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Park auto: drives from its own start pose to the park spot. Written in RED planner
 * coordinates (corner origin, from the Neuron Path Planner); Blue is the mirror image.
 * Uses everything else from PlannerAuto: init, the flywheel spin-up, and saving the pose
 * every loop, so TeleOp (and its auto aim) starts from where the robot parked.
 * ParkAutoRed / ParkAutoBlue pick the alliance.
 */
public abstract class ParkAuto extends PlannerAuto {
    @Override
    protected Pose2D startPose() {
        return red(90.01, 136, 180);
    }

    @Override
    protected void runPath() {
        localizer.setPosition(startPose());
        drive.goToPoint(red(104, 132, 180), false, MAX_SPEED);
        driveToTarget();
    }

    // Pose from RED planner coordinates (inches, degrees, corner origin). Shifts to the
    // center origin; Red keeps the numbers, Blue (alliance -1) gets y and heading flipped.
    private Pose2D red(double x, double y, double headingDeg) {
        double centerX = x - PLANNER_ORIGIN_OFFSET_IN;
        double centerY = y - PLANNER_ORIGIN_OFFSET_IN;
        return new Pose2D(centerX, alliance * centerY, Math.toRadians(alliance * headingDeg));
    }
}
