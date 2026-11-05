package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter pivot: two servos on opposite sides that turn in opposite directions.
 * Both get the same position; the right servo is set to Direction.REVERSE so the SDK
 * mirrors it (position p becomes 1 - p). RIGHT_TRIM nudges the right servo if the two
 * horns weren't mounted at exactly mirrored angles.
 *
 * Preset poses used through the match: BOTTOM, TRANSITION, SHOOT_FRONT, SHOOT_BACK.
 * Positions are @Config so they can be set live in FTC Dashboard or with the
 * "Servo Position Tuning" OpMode; copy the final values back into this file.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class ShooterPivot {
    public enum Pose {
        BOTTOM,
        TRANSITION,
        SHOOT_FRONT,
        SHOOT_BACK,
        MANUAL
    }

    private final Servo left;
    private final Servo right;
    public Pose pose = Pose.BOTTOM;

    // Placeholder positions (left servo's value). NEED TUNING on the robot.
    public static double BOTTOM_POSITION = 0.35;
    public static double TRANSITION_POSITION = 0.45;
    public static double SHOOT_FRONT_POSITION = 0.55;
    public static double SHOOT_BACK_POSITION = 0.65;

    /** Added to the right servo's position (after mirroring) to line it up with the left. */
    public static double RIGHT_TRIM = 0.0;
    /** Flip this if the pivot moves the wrong way for increasing positions. */
    public static boolean SWAP_DIRECTIONS = false;

    private double position;

    public ShooterPivot(HardwareMap hwMap, String leftName, String rightName) {
        left = hwMap.get(Servo.class, leftName);
        right = hwMap.get(Servo.class, rightName);
        left.setDirection(SWAP_DIRECTIONS ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        right.setDirection(SWAP_DIRECTIONS ? Servo.Direction.FORWARD : Servo.Direction.REVERSE);
        goTo(Pose.BOTTOM);
    }

    public ShooterPivot(HardwareMap hwMap) {
        this(hwMap, "pivotLeft", "pivotRight");
    }

    public void goTo(Pose newPose) {
        switch (newPose) {
            case BOTTOM:      apply(BOTTOM_POSITION); break;
            case TRANSITION:  apply(TRANSITION_POSITION); break;
            case SHOOT_FRONT: apply(SHOOT_FRONT_POSITION); break;
            case SHOOT_BACK:  apply(SHOOT_BACK_POSITION); break;
            case MANUAL:      apply(position); break;
        }
        pose = newPose;
    }

    public void bottom()     { goTo(Pose.BOTTOM); }
    public void transition() { goTo(Pose.TRANSITION); }
    public void shootFront() { goTo(Pose.SHOOT_FRONT); }
    public void shootBack()  { goTo(Pose.SHOOT_BACK); }

    /** Sets a raw position (left servo's value) outside the presets, e.g. while tuning. */
    public void setPosition(double newPosition) {
        apply(newPosition);
        pose = Pose.MANUAL;
    }

    /** The position last sent to the left servo (the right one gets the mirror of it). */
    public double getPosition() {
        return position;
    }

    private void apply(double newPosition) {
        position = Range.clip(newPosition, 0, 1);
        left.setPosition(position);
        right.setPosition(Range.clip(position + RIGHT_TRIM, 0, 1));
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Re-applies the current preset,
     * so preset or trim changes made in FTC Dashboard take effect right away.
     */
    public void update() {
        goTo(pose);
    }
}
