package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter pivot: two servos on opposite sides that move together.
 *
 * The right servo is inverted in hardware (set up on the servo itself) and both servos'
 * gears are centred, so both get the SAME position and the pivot turns as one piece.
 * The pivot uses the servos' full range: 1 = BOTTOM (ground / stowed), 0 = SHOOT_BACK.
 *
 * Preset poses, in order through the range: BOTTOM, TRANSITION, SHOOT_FRONT, SHOOT_BACK.
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

    /** The presets in order from ground (1) to shoot back (0). */
    public static final Pose[] PRESETS = {Pose.BOTTOM, Pose.TRANSITION, Pose.SHOOT_FRONT, Pose.SHOOT_BACK};

    private final ServoImplEx left;
    private final ServoImplEx right;
    public Pose pose = Pose.BOTTOM;
    private boolean released = false;

    // Positions sent to BOTH servos. TRANSITION and SHOOT_FRONT are placeholders: NEED TUNING.
    public static double BOTTOM_POSITION = 1.0;
    public static double TRANSITION_POSITION = 0.67;
    public static double SHOOT_FRONT_POSITION = 0.33;
    public static double SHOOT_BACK_POSITION = 0.0;

    private double position;

    /**
     * @param moveToBottom true = go to BOTTOM right away; false = start released (limp),
     *                     so nothing moves until a pose is commanded.
     */
    public ShooterPivot(HardwareMap hwMap, String leftName, String rightName, boolean moveToBottom) {
        left = hwMap.get(ServoImplEx.class, leftName);
        right = hwMap.get(ServoImplEx.class, rightName);
        left.setDirection(Servo.Direction.FORWARD);
        right.setDirection(Servo.Direction.FORWARD);
        if (moveToBottom) {
            goTo(Pose.BOTTOM);
        } else {
            release();
        }
    }

    public ShooterPivot(HardwareMap hwMap, boolean moveToBottom) {
        this(hwMap, "pivotLeft", "pivotRight", moveToBottom);
    }

    public ShooterPivot(HardwareMap hwMap) {
        this(hwMap, true);
    }

    /**
     * Turns off both servos' signal so they go limp and can be moved by hand.
     * They stay limp until goTo()/setPosition() commands a position again.
     */
    public void release() {
        left.setPwmDisable();
        right.setPwmDisable();
        released = true;
    }

    public boolean isReleased() {
        return released;
    }

    /** The position stored for a preset (MANUAL returns the current position). */
    public double presetPosition(Pose p) {
        switch (p) {
            case BOTTOM:      return BOTTOM_POSITION;
            case TRANSITION:  return TRANSITION_POSITION;
            case SHOOT_FRONT: return SHOOT_FRONT_POSITION;
            case SHOOT_BACK:  return SHOOT_BACK_POSITION;
            default:          return position;
        }
    }

    /** Saves a position into a preset (lasts until the robot restarts; copy it into this file). */
    public static void savePreset(Pose p, double value) {
        switch (p) {
            case BOTTOM:      BOTTOM_POSITION = value; break;
            case TRANSITION:  TRANSITION_POSITION = value; break;
            case SHOOT_FRONT: SHOOT_FRONT_POSITION = value; break;
            case SHOOT_BACK:  SHOOT_BACK_POSITION = value; break;
            default: break;
        }
    }

    public void goTo(Pose newPose) {
        apply(presetPosition(newPose));
        pose = newPose;
    }

    public void bottom()     { goTo(Pose.BOTTOM); }
    public void transition() { goTo(Pose.TRANSITION); }
    public void shootFront() { goTo(Pose.SHOOT_FRONT); }
    public void shootBack()  { goTo(Pose.SHOOT_BACK); }

    /** Moves both servos to a position outside the presets, e.g. while tuning. */
    public void setPosition(double newPosition) {
        apply(newPosition);
        pose = Pose.MANUAL;
    }

    /** The position last sent to both servos. */
    public double getPosition() {
        return position;
    }

    private void apply(double newPosition) {
        position = Range.clip(newPosition, 0, 1);
        left.setPosition(position);
        right.setPosition(position);
        left.setPwmEnable();
        right.setPwmEnable();
        released = false;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Re-applies the current pose,
     * so preset changes made in FTC Dashboard take effect right away. Does nothing
     * while released.
     */
    public void update() {
        if (!released) goTo(pose);
    }
}
