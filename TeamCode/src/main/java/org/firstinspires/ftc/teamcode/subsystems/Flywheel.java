package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Subsystem for a single-motor flywheel with PIDF velocity control.
 * Speed is read in encoder ticks per second from the encoder cable plugged into
 * the flywheel motor's own encoder port. Power each loop = F (kF * target + kS)
 * + P + I + D on the filtered velocity error, with a low-pass filter on the
 * derivative, the same structure as the team's previous two-motor Shooter.
 * Gains are @Config so they show up in FTC Dashboard; copy the tuned values back
 * into this file, since Dashboard changes don't persist.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class Flywheel {
    public enum State {
        SPINNING,
        STOPPED
    }

    private final DcMotorEx motor;
    public State state = State.STOPPED;

    public static boolean REVERSED = true;

    // ---- tunable PIDF velocity gains (power units per tick/sec) ------------------
    // Tuned on the robot, starting from the previous two-motor Shooter's gains
    // (kP 0.02, kF 0.00085) for the same flywheel.
    public static double kP = 0.03, kI = 0.0, kD = 0.0;
    /** F (feedforward): power per tick/sec of target. Tune first, with kP/kI/kD at 0. */
    public static double kF = 0.00085;
    /** Static feedforward: constant power to overcome friction whenever the target isn't 0. */
    public static double kS = 0.0;
    /** Acceleration feedforward: power per (tick/sec per second) of target change. Adds power
     *  while the target rises and removes it while it falls, so the flywheel eases into peaks
     *  instead of overshooting. Start at 0, try 0.0005, and raise in small steps (up to ~0.003). */
    public static double kA = 0.0;
    /** Cap on the kA term, so a sudden target jump (e.g. 0 -> 500) can't spike the power. */
    public static double MAX_ACCEL_POWER = 0.3;
    /** Low-pass filter on the derivative term, 0 to <1. Higher = smoother D. */
    public static double D_FILTER = 0.8;

    // ---- dampening (active slowing down) -----------------------------------------
    /** Max reverse power the controller may apply when the flywheel is faster than the
     *  target, e.g. when the target drops. 0 = coast down only. Keep it small to spare the gearbox. */
    public static double MAX_BRAKE_POWER = 0.12;
    /** Only brake when the flywheel is more than this many ticks/sec above the target.
     *  Inside this band power can't go below 0, so encoder noise near the target
     *  can't flicker the motor between driving and braking. */
    public static double BRAKE_THRESHOLD = 20;
    /** Brake power ramps from 0 at BRAKE_THRESHOLD up to MAX_BRAKE_POWER over this many
     *  more ticks/sec of overspeed, so braking eases in instead of switching on and off. */
    public static double BRAKE_RAMP = 60;
    /** When the target is 0: true = motor brakes to a stop, false = flywheel coasts. */
    public static boolean BRAKE_WHEN_STOPPED = false;

    /** Low-pass filter on the measured speed, 0 to <1. 0 = raw encoder reading;
     *  higher = smoother but slower to react. Each loop keeps this fraction of the old value. */
    public static double VELOCITY_FILTER = 0.7;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double targetTicksPerSec = 0;
    private double lastTargetTicksPerSec = 0;
    private double filteredTicksPerSec = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double integral = 0;
    private double lastPower = 0;
    private boolean coasting = false;

    public Flywheel(HardwareMap hwMap, String motorName) {
        motor = hwMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Our own loop sets power; the encoder still counts in this mode
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        stop();
    }

    public Flywheel(HardwareMap hwMap) {
        this(hwMap, "flywheel");
    }

    public void setTargetTicksPerSec(double ticksPerSec) {
        targetTicksPerSec = Math.max(0, ticksPerSec);
        state = targetTicksPerSec > 0 ? State.SPINNING : State.STOPPED;
        coasting = false;
    }

    /**
     * Lowers the target, but lets the flywheel coast down to it with 0 power (as if the
     * target were 0) instead of braking, to save battery. Normal PIDF takes over once the
     * speed has dropped to the new target.
     */
    public void coastDownTo(double ticksPerSec) {
        setTargetTicksPerSec(ticksPerSec);
        coasting = true;
    }

    public boolean isCoasting() {
        return coasting;
    }

    public void stop() {
        setTargetTicksPerSec(0);
    }

    public double getTargetTicksPerSec() {
        return targetTicksPerSec;
    }

    /** Filtered speed used by the controller (updated in update()). */
    public double getTicksPerSec() {
        return filteredTicksPerSec;
    }

    /** Unfiltered encoder speed, for comparing against the filtered value while tuning. */
    public double getRawTicksPerSec() {
        return motor.getVelocity();
    }

    public double getError() {
        return targetTicksPerSec - getTicksPerSec();
    }

    public double getPower() {
        return lastPower;
    }

    public boolean atSpeed(double toleranceTicksPerSec) {
        return targetTicksPerSec > 0 && Math.abs(getError()) < toleranceTicksPerSec;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Runs the velocity controller.
     */
    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        double filter = Range.clip(VELOCITY_FILTER, 0, 0.99);
        filteredTicksPerSec = filter * filteredTicksPerSec + (1 - filter) * getRawTicksPerSec();

        motor.setZeroPowerBehavior(BRAKE_WHEN_STOPPED
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);

        double error = getError();
        double targetRate = (dt > 0) ? (targetTicksPerSec - lastTargetTicksPerSec) / dt : 0;
        lastTargetTicksPerSec = targetTicksPerSec;

        // Coasting down after coastDownTo(): stay at 0 power until we reach the new target
        if (coasting && filteredTicksPerSec <= targetTicksPerSec) {
            coasting = false;
        }

        if (targetTicksPerSec <= 0 || coasting) {
            integral = 0;
            filteredDerivative = 0;
            lastError = error;
            lastPower = 0;
            motor.setPower(0);
            return;
        }

        double rawDerivative = (dt > 0) ? (error - lastError) / dt : 0;
        double dFilter = Range.clip(D_FILTER, 0, 0.99);
        filteredDerivative = dFilter * filteredDerivative + (1 - dFilter) * rawDerivative;
        lastError = error;

        // Brake eases in: 0 up to BRAKE_THRESHOLD overspeed, then ramps to MAX_BRAKE_POWER
        double overspeed = -error;
        double brakeFraction = (BRAKE_RAMP > 0)
                ? Range.clip((overspeed - BRAKE_THRESHOLD) / BRAKE_RAMP, 0, 1)
                : (overspeed > BRAKE_THRESHOLD ? 1 : 0);
        double minPower = -MAX_BRAKE_POWER * brakeFraction;

        // Only build up I while the output isn't already maxed out, so it can't wind up
        if (lastPower < 1 && lastPower > minPower) {
            integral += error * dt;
        }

        double accelPower = Range.clip(kA * targetRate, -MAX_ACCEL_POWER, MAX_ACCEL_POWER);
        double power = kF * targetTicksPerSec + kS + accelPower
                + kP * error + kI * integral + kD * filteredDerivative;
        lastPower = Range.clip(power, minPower, 1);
        motor.setPower(lastPower);
    }
}
