package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shoot macro. Non-blocking: call start(), then update() every loop until isBusy()
 * is false. It only sets targets/states on the other subsystems; the OpMode still
 * calls their own update() every loop.
 *
 * Sequence:
 * SPIN_UP     -> flywheel to the shot speed; wait until it's at speed (or SPIN_UP_TIMEOUT_MS)
 * GATE_DELAY  -> transfer starts feeding; GATE_DELAY_MS later the gate opens
 * FEEDING     -> transfer keeps feeding until FEED_MS after it started
 * then the gate closes, the transfer stops, and the flywheel target drops by
 * SPEED_DROP_AFTER_SHOT. The flywheel coasts down to that lower speed with 0 power
 * (no braking) to save battery, then holds it.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class Shooter {
    public enum State {
        IDLE,
        SPIN_UP,
        GATE_DELAY,
        FEEDING
    }

    private final Flywheel flywheel;
    private final Transfer transfer;
    private final Gate gate;
    public State state = State.IDLE;

    /** Default flywheel speed for a shot, in ticks/sec. */
    public static double SHOOT_TICKS_PER_SEC = 600;
    /** Flywheel counts as ready when within this many ticks/sec of the target. */
    public static double AT_SPEED_TOLERANCE_TICKS_PER_SEC = 25;
    /** Shoot anyway if the flywheel hasn't reached speed by then, so auto can't get stuck. */
    public static double SPIN_UP_TIMEOUT_MS = 2500;
    /** Delay between the transfer starting and the gate opening. */
    public static double GATE_DELAY_MS = 200;
    /** How long the transfer runs, counted from when it starts. */
    public static double FEED_MS = 4000;
    /** After the shot the flywheel target drops by this much (and coasts down to it). */
    public static double SPEED_DROP_AFTER_SHOT = 500;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private double shotTargetTicksPerSec = 0;

    public Shooter(Flywheel flywheel, Transfer transfer, Gate gate) {
        this.flywheel = flywheel;
        this.transfer = transfer;
        this.gate = gate;
    }

    /** Starts a shot at the default speed. Ignored if a shot is already running. */
    public void start() {
        start(SHOOT_TICKS_PER_SEC);
    }

    /** Starts a shot at the given flywheel speed. Ignored if a shot is already running. */
    public void start(double targetTicksPerSec) {
        if (isBusy()) return;
        shotTargetTicksPerSec = targetTicksPerSec;
        flywheel.setTargetTicksPerSec(targetTicksPerSec);
        gate.close();
        setState(State.SPIN_UP);
    }

    /** Stops the shot immediately and closes everything up. */
    public void abort() {
        if (isBusy()) {
            finish();
        }
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Advances the macro.
     */
    public void update() {
        switch (state) {
            case SPIN_UP:
                if (flywheel.atSpeed(AT_SPEED_TOLERANCE_TICKS_PER_SEC)
                        || stateTimer.milliseconds() >= SPIN_UP_TIMEOUT_MS) {
                    transfer.feed();
                    feedTimer.reset();
                    setState(State.GATE_DELAY);
                }
                break;
            case GATE_DELAY:
                if (stateTimer.milliseconds() >= GATE_DELAY_MS) {
                    gate.open();
                    setState(State.FEEDING);
                }
                break;
            case FEEDING:
                if (feedTimer.milliseconds() >= FEED_MS) {
                    finish();
                }
                break;
            case IDLE:
                break;
        }
    }

    private void finish() {
        gate.close();
        transfer.stop();
        flywheel.coastDownTo(shotTargetTicksPerSec - SPEED_DROP_AFTER_SHOT);
        setState(State.IDLE);
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
}
