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
    // Starting values come from the previous two-motor Shooter (kP 0.02, kF 0.00085)
    // for the same flywheel. kF is per-motor so it carries over to one motor. With
    // one motor the flywheel gets half the torque, so the same kP reacts about half
    // as fast; raise kP toward 0.04 for the same response if it stays stable.
    public static double kP = 0.02, kI = 0.0, kD = 0.0;
    /** F (feedforward): power per tick/sec of target. Tune first, with kP/kI/kD at 0. */
    public static double kF = 0.00085;
    /** Static feedforward: constant power to overcome friction whenever the target isn't 0. */
    public static double kS = 0.0;
    /** Low-pass filter on the derivative term, 0 to <1. Higher = smoother D. */
    public static double D_FILTER = 0.8;

    // ---- dampening (active slowing down) -----------------------------------------
    /** Max reverse power the controller may apply when the flywheel is faster than the
     *  target, e.g. when the target drops. 0 = coast down only. Keep it small to spare the gearbox. */
    public static double MAX_BRAKE_POWER = 0.05;
    /** Only brake when the flywheel is more than this many ticks/sec above the target.
     *  Inside this band power can't go below 0, so encoder noise near the target
     *  can't flicker the motor between driving and braking. */
    public static double BRAKE_THRESHOLD = 40;
    /** When the target is 0: true = motor brakes to a stop, false = flywheel coasts. */
    public static boolean BRAKE_WHEN_STOPPED = false;

    /** Low-pass filter on the measured speed, 0 to <1. 0 = raw encoder reading;
     *  higher = smoother but slower to react. Each loop keeps this fraction of the old value. */
    public static double VELOCITY_FILTER = 0.7;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double targetTicksPerSec = 0;
    private double filteredTicksPerSec = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double integral = 0;
    private double lastPower = 0;

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
        if (targetTicksPerSec <= 0) {
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

        // Only build up I while the output isn't already maxed out, so it can't wind up
        if (lastPower < 1 && lastPower > -MAX_BRAKE_POWER) {
            integral += error * dt;
        }

        double power = kF * targetTicksPerSec + kS
                + kP * error + kI * integral + kD * filteredDerivative;
        // Allow a little reverse power only when clearly above target; otherwise just coast
        double minPower = (-error > BRAKE_THRESHOLD) ? -MAX_BRAKE_POWER : 0;
        lastPower = Range.clip(power, minPower, 1);
        motor.setPower(lastPower);
    }
}
