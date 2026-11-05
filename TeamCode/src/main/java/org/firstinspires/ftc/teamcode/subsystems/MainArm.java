package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Main arm pivot: one 117 RPM motor with its encoder plugged into its own port, held at
 * a position by our own PID loop.
 *
 * The arm moves slowly and smoothly: setTarget() sets the goal, and each loop the
 * PID setpoint walks toward that goal at no more than MAX_VELOCITY_TICKS_PER_SEC. The
 * PID then follows the setpoint. A gravity term (kG * cos(arm angle)) holds the arm up,
 * so P doesn't have to fight its weight on its own.
 *
 * Power each loop = kG * cos(angle) + kS * sign(error) + P + I + D on (setpoint - position),
 * capped at +/- MAX_POWER.
 *
 * The encoder resets to 0 when the subsystem is built, so the arm must be resting at
 * the same place (its bottom/stowed position) every time an OpMode inits.
 * Gains are @Config for FTC Dashboard; copy tuned values back into this file.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class MainArm {
    public enum State {
        MOVING,
        HOLDING,
        DISABLED
    }

    private final DcMotorEx motor;
    public State state = State.HOLDING;

    public static boolean REVERSED = false;

    // ---- geometry ----------------------------------------------------------------
    /** Encoder ticks per output-shaft revolution. goBILDA 5203 117 RPM (50.9:1) = 1425.1. */
    public static double TICKS_PER_REV = 1425.1;
    /** Extra gearing between the motor shaft and the arm (arm turns / motor turns inverted,
     *  e.g. 2.0 if a 2:1 reduction means the motor turns twice per arm turn). 1 = direct drive. */
    public static double EXTERNAL_GEAR_RATIO = 1.0;
    /** Arm angle from horizontal, in degrees, when the encoder reads 0 (the resting position).
     *  E.g. -30 if the arm rests 30 degrees below horizontal. Used for the gravity term. */
    public static double ANGLE_AT_ZERO_DEG = 0.0;

    // ---- soft limits (ticks) -----------------------------------------------------
    /** setTarget() clips the goal to this range. Set them once you know the arm's travel. */
    public static double MIN_TICKS = 0;
    public static double MAX_TICKS = 1000;

    // ---- tunable PID (power per tick of error) -----------------------------------
    // NEEDS TUNING. Order: kG first (arm floats where you leave it), then kP, then kD,
    // and kI only if it sits short of the target. Use the "Main Arm Tuning" OpMode.
    public static double kP = 0.0, kI = 0.0, kD = 0.0;
    /** Gravity feedforward: power needed to hold the arm level (horizontal). */
    public static double kG = 0.0;
    /** Static friction feedforward: small push in the direction of the error, only when
     *  the error is bigger than POSITION_TOLERANCE_TICKS. */
    public static double kS = 0.0;
    /** Low-pass filter on the derivative term, 0 to <1. Higher = smoother D. */
    public static double D_FILTER = 0.7;
    /** I only builds up within this many ticks of the setpoint, so big moves don't wind it up. */
    public static double I_ZONE_TICKS = 40;

    // ---- motion limits -----------------------------------------------------------
    /** Max power the controller may command, either direction. Keep it low for gentle moves. */
    public static double MAX_POWER = 0.6;
    /** How fast the setpoint walks toward the goal. Lower = slower, smoother arm. */
    public static double MAX_VELOCITY_TICKS_PER_SEC = 600;
    /** Arm counts as "at target" within this many ticks (and when the setpoint has arrived). */
    public static double POSITION_TOLERANCE_TICKS = 10;

    private final ElapsedTime loopTimer = new ElapsedTime();

    private double goalTicks = 0;
    private double setpointTicks = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double integral = 0;
    private double lastPower = 0;

    public MainArm(HardwareMap hwMap, String motorName) {
        motor = hwMap.get(DcMotorEx.class, motorName);
        motor.setDirection(REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Our own loop sets power; the encoder still counts in this mode
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    public MainArm(HardwareMap hwMap) {
        this(hwMap, "mainArm");
    }

    /** Sets the goal position in encoder ticks (clipped to MIN_TICKS..MAX_TICKS). */
    public void setTarget(double ticks) {
        goalTicks = Range.clip(ticks, MIN_TICKS, MAX_TICKS);
        if (state == State.DISABLED) {
            // Start the profile from where the arm actually is, not where it was left
            setpointTicks = getPosition();
            resetController();
        }
        state = State.MOVING;
    }

    /** Moves the goal by deltaTicks, e.g. from a joystick. */
    public void adjustTarget(double deltaTicks) {
        setTarget(goalTicks + deltaTicks);
    }

    /** Cuts power (the motor brakes) until setTarget() is called again. */
    public void disable() {
        state = State.DISABLED;
        motor.setPower(0);
        lastPower = 0;
    }

    /** Re-zeroes the encoder at the arm's current spot, e.g. after it slipped. */
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        goalTicks = 0;
        setpointTicks = 0;
        resetController();
    }

    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public double getGoal() {
        return goalTicks;
    }

    /** The profiled setpoint the PID is following right now. */
    public double getSetpoint() {
        return setpointTicks;
    }

    public double getError() {
        return setpointTicks - getPosition();
    }

    public double getPower() {
        return lastPower;
    }

    /** Arm angle from horizontal, in degrees. */
    public double getAngleDeg() {
        return ANGLE_AT_ZERO_DEG + getPosition() / ticksPerArmDegree();
    }

    public boolean atTarget() {
        return setpointTicks == goalTicks
                && Math.abs(goalTicks - getPosition()) < POSITION_TOLERANCE_TICKS;
    }

    public static double degreesToTicks(double armDegrees) {
        return armDegrees * ticksPerArmDegree();
    }

    private static double ticksPerArmDegree() {
        return TICKS_PER_REV * EXTERNAL_GEAR_RATIO / 360.0;
    }

    private void resetController() {
        integral = 0;
        filteredDerivative = 0;
        lastError = 0;
        loopTimer.reset();
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Runs the position controller.
     */
    public void update() {
        // Capped so a long gap (e.g. init -> start) can't make the setpoint jump
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        if (state == State.DISABLED) {
            return;
        }

        // Walk the setpoint toward the goal at no more than MAX_VELOCITY_TICKS_PER_SEC
        double maxStep = Math.max(0, MAX_VELOCITY_TICKS_PER_SEC) * dt;
        setpointTicks += Range.clip(goalTicks - setpointTicks, -maxStep, maxStep);
        state = (setpointTicks == goalTicks) ? State.HOLDING : State.MOVING;

        double position = getPosition();
        double error = setpointTicks - position;

        double rawDerivative = (dt > 0) ? (error - lastError) / dt : 0;
        double dFilter = Range.clip(D_FILTER, 0, 0.99);
        filteredDerivative = dFilter * filteredDerivative + (1 - dFilter) * rawDerivative;
        lastError = error;

        // I only inside the I-zone, and only while the output isn't maxed out
        if (Math.abs(error) < I_ZONE_TICKS && Math.abs(lastPower) < MAX_POWER) {
            integral += error * dt;
        } else if (Math.abs(error) >= I_ZONE_TICKS) {
            integral = 0;
        }

        double gravity = kG * Math.cos(Math.toRadians(getAngleDeg()));
        double friction = Math.abs(error) > POSITION_TOLERANCE_TICKS ? kS * Math.signum(error) : 0;
        double power = gravity + friction + kP * error + kI * integral + kD * filteredDerivative;

        double maxPower = Range.clip(MAX_POWER, 0, 1);
        lastPower = Range.clip(power, -maxPower, maxPower);
        motor.setPower(lastPower);
    }
}
