package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Up/down macro for the main arm and shooter pivot. Non-blocking: call up() or down(),
 * then update() every loop. It only sets targets on the arm and pivot; the OpMode still
 * calls their own update() every loop.
 *
 * UP:
 *   1. Pivot -> TRANSITION. Wait until it gets there.
 *   2. At the same time: arm -> UP_ARM_TICKS and pivot -> SHOOT_FRONT.
 *      Done when both have arrived (or after ARM_TIMEOUT_MS).
 * DOWN:
 *   1. Arm -> DOWN_ARM_TICKS (pivot stays where it is). Wait until the arm reaches its
 *      target (or ARM_TIMEOUT_MS).
 *   2. Pivot -> TRANSITION. Wait until it gets there.
 *   3. Pivot -> BOTTOM. Wait until it gets there, then done.
 *
 * "Gets there" for the pivot = its speed-limited move has finished, plus PIVOT_SETTLE_MS
 * for the servos to catch up (they can't report their real position). PIVOT_TIMEOUT_MS
 * moves on anyway so the macro can't get stuck.
 * Starting one direction while the other is running switches to it straight away.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class ArmPivotMacro {
    public enum State {
        IDLE,
        UP_PIVOT_TO_TRANSITION,
        UP_ARM_AND_PIVOT,
        DOWN_ARM,
        DOWN_PIVOT_TO_TRANSITION,
        DOWN_PIVOT_TO_BOTTOM
    }

    /** Arm target at the top of the UP macro, in encoder ticks. */
    public static double UP_ARM_TICKS = 800;
    /** Arm target at the end of the DOWN macro (0 = where it rested at init). */
    public static double DOWN_ARM_TICKS = 0;
    /** Extra wait after a pivot move finishes, for the servos to catch up. */
    public static double PIVOT_SETTLE_MS = 200;
    /** Move on anyway if the pivot move hasn't finished by then. */
    public static double PIVOT_TIMEOUT_MS = 3000;
    /** Move on anyway if the arm hasn't reached its target by then, so it can't get stuck. */
    public static double ARM_TIMEOUT_MS = 4000;

    private final MainArm arm;
    private final ShooterPivot pivot;
    public State state = State.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    public ArmPivotMacro(MainArm arm, ShooterPivot pivot) {
        this.arm = arm;
        this.pivot = pivot;
    }

    /** Starts the UP sequence. */
    public void up() {
        pivot.transition();
        setState(State.UP_PIVOT_TO_TRANSITION);
    }

    /** Starts the DOWN sequence. */
    public void down() {
        arm.setTarget(DOWN_ARM_TICKS);
        setState(State.DOWN_ARM);
    }

    /** Stops the macro where it is; the arm and pivot hold their current targets. */
    public void cancel() {
        setState(State.IDLE);
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Advances the macro.
     */
    public void update() {
        double ms = stateTimer.milliseconds();
        switch (state) {
            case UP_PIVOT_TO_TRANSITION:
                if (pivotThere(ms)) {
                    arm.setTarget(UP_ARM_TICKS);
                    pivot.shootFront();
                    setState(State.UP_ARM_AND_PIVOT);
                }
                break;
            case UP_ARM_AND_PIVOT:
                // Done once both the arm and the (slower) pivot have arrived
                if ((arm.atTarget() && pivotThere(ms)) || ms >= ARM_TIMEOUT_MS) {
                    setState(State.IDLE);
                }
                break;
            case DOWN_ARM:
                if (arm.atTarget() || ms >= ARM_TIMEOUT_MS) {
                    pivot.transition();
                    setState(State.DOWN_PIVOT_TO_TRANSITION);
                }
                break;
            case DOWN_PIVOT_TO_TRANSITION:
                if (pivotThere(ms)) {
                    pivot.bottom();
                    setState(State.DOWN_PIVOT_TO_BOTTOM);
                }
                break;
            case DOWN_PIVOT_TO_BOTTOM:
                if (pivotThere(ms)) {
                    setState(State.IDLE);
                }
                break;
            case IDLE:
                break;
        }
    }

    /** The pivot's speed-limited move has finished and settled, or it's taking too long. */
    private boolean pivotThere(double ms) {
        return pivot.isSettled(PIVOT_SETTLE_MS) || ms >= PIVOT_TIMEOUT_MS;
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
}
