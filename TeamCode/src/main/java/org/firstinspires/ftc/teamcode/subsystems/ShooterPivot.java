package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Shooter pivot: two servos on opposite sides that move together.
 *
 * The right servo is inverted in hardware (set up on the servo itself) and both servos'
 * gears are centred, so both get the SAME position and the pivot turns as one piece.
 * The pivot uses the servos' full range: 1 = BOTTOM (ground / stowed), 0 = SHOOT_BACK.
 *
 * Speed limit: instead of jumping straight to a new position, the commanded position
 * walks toward it at MAX_SPEED_PER_SEC each loop, so the servos move slower than their
 * full speed. This needs update() called every loop. 0 = no limit (instant moves).
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
    public static double TRANSITION_POSITION = 0.83;
    public static double SHOOT_FRONT_POSITION = 0.3;
    public static double SHOOT_BACK_POSITION = 0.0;

    /** Max change in servo position per second. About 60% of an Axon MAX's full speed
     *  (it covers the full 0..1 range in roughly 0.7 s), i.e. about 40% slower. This is an
     *  estimate: lower it to slow down more, raise it to speed up, 0 = full servo speed. */
    public static double MAX_SPEED_PER_SEC = 0.85;

    /** Where the servos are headed. */
    private double target;
    /** What's being sent to the servos right now (walks toward target). */
    private double position;
    private boolean moving = false;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime arrivedTimer = new ElapsedTime();

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
            // First command: nothing to walk from, so go straight there
            jumpTo(BOTTOM_POSITION);
            pose = Pose.BOTTOM;
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
        moving = false;
    }

    public boolean isReleased() {
        return released;
    }

    /** The position stored for a preset (MANUAL returns the current target). */
    public double presetPosition(Pose p) {
        switch (p) {
            case BOTTOM:      return BOTTOM_POSITION;
            case TRANSITION:  return TRANSITION_POSITION;
            case SHOOT_FRONT: return SHOOT_FRONT_POSITION;
            case SHOOT_BACK:  return SHOOT_BACK_POSITION;
            default:          return target;
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
        moveTo(presetPosition(newPose));
        pose = newPose;
    }

    public void bottom()     { goTo(Pose.BOTTOM); }
    public void transition() { goTo(Pose.TRANSITION); }
    public void shootFront() { goTo(Pose.SHOOT_FRONT); }
    public void shootBack()  { goTo(Pose.SHOOT_BACK); }

    /** Moves both servos to a position outside the presets, e.g. while tuning. */
    public void setPosition(double newPosition) {
        moveTo(newPosition);
        pose = Pose.MANUAL;
    }

    /** What's being sent to both servos right now (partway there during a slow move). */
    public double getPosition() {
        return position;
    }

    /** Where the servos are headed. */
    public double getTarget() {
        return target;
    }

    /** True while the commanded position is still walking toward the target. */
    public boolean isMoving() {
        return moving;
    }

    /** True once the move has finished and at least settleMs has passed since. */
    public boolean isSettled(double settleMs) {
        return !released && !moving && arrivedTimer.milliseconds() >= settleMs;
    }

    private void moveTo(double newTarget) {
        target = Range.clip(newTarget, 0, 1);
        if (released || MAX_SPEED_PER_SEC <= 0) {
            // Coming back from limp, the real servo position is unknown, so there's
            // nothing to walk from: go straight there
            jumpTo(target);
            return;
        }
        if (position != target) {
            moving = true;
            loopTimer.reset();
        }
    }

    private void jumpTo(double newPosition) {
        target = Range.clip(newPosition, 0, 1);
        position = target;
        write();
        moving = false;
        arrivedTimer.reset();
        loopTimer.reset();
    }

    private void write() {
        left.setPosition(position);
        right.setPosition(position);
        left.setPwmEnable();
        right.setPwmEnable();
        released = false;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Walks the servos toward the
     * target at MAX_SPEED_PER_SEC. Preset changes made in FTC Dashboard take effect
     * right away. Does nothing while released.
     */
    public void update() {
        // Capped so a long gap between loops can't make it jump
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();
        if (released) return;

        if (pose != Pose.MANUAL) {
            double presetTarget = Range.clip(presetPosition(pose), 0, 1);
            if (presetTarget != target) moveTo(presetTarget);
        }

        double maxStep = MAX_SPEED_PER_SEC * dt;
        if (MAX_SPEED_PER_SEC <= 0 || Math.abs(target - position) <= maxStep) {
            position = target; // exact, so the arrival check below always triggers
        } else {
            position += Math.signum(target - position) * maxStep;
        }
        write();

        if (moving && position == target) {
            moving = false;
            arrivedTimer.reset();
        }
    }
}
