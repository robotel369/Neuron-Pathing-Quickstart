package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Shoot macro. Non-blocking: call start(), then update() every loop until isBusy()
 * is false. It only sets targets/states on the other subsystems; the OpMode still
 * calls their own update() every loop.
 *
 * Sequence:
 * SPIN_UP       -> flywheel to the shot speed; wait until it's at speed (or SPIN_UP_TIMEOUT_MS),
 *                  then the gate opens
 * OUTTAKE_PULSE -> intake runs out at OUTTAKE_PULSE_POWER for OUTTAKE_PULSE_MS
 * INTAKE_PULSE  -> intake runs in at INTAKE_PULSE_POWER for INTAKE_PULSE_MS
 * PAUSE         -> intake stops for PAUSE_AFTER_INTAKE_MS
 * The three steps repeat FEED_CYCLES times. Then the gate closes, the intake goes back
 * to the power it had before the shot, and the flywheel target drops by
 * SPEED_DROP_AFTER_SHOT. The flywheel coasts down to that lower speed with 0 power
 * (no braking) to save battery, then holds it.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class Shooter {
    public enum State {
        IDLE,
        SPIN_UP,
        OUTTAKE_PULSE,
        INTAKE_PULSE,
        PAUSE
    }

    private final Flywheel flywheel;
    private final Intake intake;
    private final Gate gate;
    public State state = State.IDLE;

    /** Default flywheel speed for a shot, in ticks/sec. */
    public static double SHOOT_TICKS_PER_SEC = 1085;
    /** Flywheel counts as ready when within this many ticks/sec of the target. */
    public static double AT_SPEED_TOLERANCE_TICKS_PER_SEC = 25;
    /** Shoot anyway if the flywheel hasn't reached speed by then, so auto can't get stuck. */
    public static double SPIN_UP_TIMEOUT_MS = 2500;
    /** Intake power and duration of the outward pulse in each feed cycle. */
    public static double OUTTAKE_PULSE_POWER = -0.6;
    public static double OUTTAKE_PULSE_MS = 100;
    /** Intake power and duration of the inward pulse in each feed cycle. */
    public static double INTAKE_PULSE_POWER = 1.0;
    public static double INTAKE_PULSE_MS = 300;
    /** How long the intake stops after each inward pulse. */
    public static double PAUSE_AFTER_INTAKE_MS = 400;
    /** How many out-in-pause cycles to run once the gate is open. */
    public static int FEED_CYCLES = 3;
    /** After the shot the flywheel target drops by this much (and coasts down to it). */
    public static double SPEED_DROP_AFTER_SHOT = 150;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private double shotTargetTicksPerSec = 0;
    private double intakePowerBeforeShot = 0;
    private int cyclesDone = 0;

    public Shooter(Flywheel flywheel, Intake intake, Gate gate) {
        this.flywheel = flywheel;
        this.intake = intake;
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
        intakePowerBeforeShot = intake.getPower();
        cyclesDone = 0;
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
                    gate.open();
                    intake.setPower(OUTTAKE_PULSE_POWER);
                    setState(State.OUTTAKE_PULSE);
                }
                break;
            case OUTTAKE_PULSE:
                if (stateTimer.milliseconds() >= OUTTAKE_PULSE_MS) {
                    intake.setPower(INTAKE_PULSE_POWER);
                    setState(State.INTAKE_PULSE);
                }
                break;
            case INTAKE_PULSE:
                if (stateTimer.milliseconds() >= INTAKE_PULSE_MS) {
                    intake.setPower(0);
                    setState(State.PAUSE);
                }
                break;
            case PAUSE:
                if (stateTimer.milliseconds() >= PAUSE_AFTER_INTAKE_MS) {
                    cyclesDone++;
                    if (cyclesDone >= FEED_CYCLES) {
                        finish();
                    } else {
                        intake.setPower(OUTTAKE_PULSE_POWER);
                        setState(State.OUTTAKE_PULSE);
                    }
                }
                break;
            case IDLE:
                break;
        }
    }

    private void finish() {
        gate.close();
        intake.setPower(intakePowerBeforeShot);
        flywheel.coastDownTo(shotTargetTicksPerSec - SPEED_DROP_AFTER_SHOT);
        setState(State.IDLE);
    }

    private void setState(State newState) {
        state = newState;
        stateTimer.reset();
    }
}
