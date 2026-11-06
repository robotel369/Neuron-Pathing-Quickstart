package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Shooter pivot: two servos on opposite sides that move together.
 *
 * The right servo is inverted in hardware (set up on the servo itself), so both servos get
 * the SAME change in position and the pivot still turns as one: 0.6 -> 0.7 on both.
 *
 * Each servo has a ZERO position (its raw servo value when the pivot is at offset 0).
 * A pose is an OFFSET from there, added to both:
 *   left  = LEFT_ZERO  + offset
 *   right = RIGHT_ZERO + offset
 * e.g. with both zeros at 0.5, offset 0.2 puts both servos at 0.7.
 * Servo positions only go from 0 to 1, so the zeros sit mid-range to leave room both ways.
 * The offset is limited so neither servo goes past 0 or 1, which keeps them together.
 *
 * Preset poses used through the match: BOTTOM, TRANSITION, SHOOT_FRONT, SHOOT_BACK.
 * Find the values with the "Shooter Pivot Tuning" OpMode and copy them into this file.
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

    /** Raw servo positions at offset 0. Adjust one of them if the two sides don't line up. */
    public static double LEFT_ZERO = 0.5;
    public static double RIGHT_ZERO = 0.5;

    // Preset offsets. Placeholders: NEED TUNING on the robot.
    public static double BOTTOM_OFFSET = 0.15;
    public static double TRANSITION_OFFSET = 0.05;
    public static double SHOOT_FRONT_OFFSET = -0.05;
    public static double SHOOT_BACK_OFFSET = -0.15;

    private double offset;

    public ShooterPivot(HardwareMap hwMap, String leftName, String rightName) {
        left = hwMap.get(Servo.class, leftName);
        right = hwMap.get(Servo.class, rightName);
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        goTo(Pose.BOTTOM);
    }

    public ShooterPivot(HardwareMap hwMap) {
        this(hwMap, "pivotLeft", "pivotRight");
    }

    /** The offset stored for a preset (MANUAL returns the current offset). */
    public double presetOffset(Pose p) {
        switch (p) {
            case BOTTOM:      return BOTTOM_OFFSET;
            case TRANSITION:  return TRANSITION_OFFSET;
            case SHOOT_FRONT: return SHOOT_FRONT_OFFSET;
            case SHOOT_BACK:  return SHOOT_BACK_OFFSET;
            default:          return offset;
        }
    }

    public void goTo(Pose newPose) {
        apply(presetOffset(newPose));
        pose = newPose;
    }

    public void bottom()     { goTo(Pose.BOTTOM); }
    public void transition() { goTo(Pose.TRANSITION); }
    public void shootFront() { goTo(Pose.SHOOT_FRONT); }
    public void shootBack()  { goTo(Pose.SHOOT_BACK); }

    /** Moves both servos to an offset outside the presets, e.g. while tuning. */
    public void setOffset(double newOffset) {
        apply(newOffset);
        pose = Pose.MANUAL;
    }

    /** Current offset from the zeros (after limiting). */
    public double getOffset() {
        return offset;
    }

    public double getLeftPosition() {
        return LEFT_ZERO + offset;
    }

    public double getRightPosition() {
        return RIGHT_ZERO + offset;
    }

    /** Smallest and largest offsets that keep both servos inside 0..1. */
    public static double minOffset() {
        return Math.max(-LEFT_ZERO, -RIGHT_ZERO);
    }

    public static double maxOffset() {
        return Math.min(1 - LEFT_ZERO, 1 - RIGHT_ZERO);
    }

    private void apply(double newOffset) {
        offset = Math.max(minOffset(), Math.min(maxOffset(), newOffset));
        left.setPosition(getLeftPosition());
        right.setPosition(getRightPosition());
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Re-applies the current pose,
     * so preset or zero changes made in FTC Dashboard take effect right away.
     */
    public void update() {
        goTo(pose);
    }
}
