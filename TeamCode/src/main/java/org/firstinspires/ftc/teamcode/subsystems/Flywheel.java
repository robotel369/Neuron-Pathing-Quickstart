package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.neuronpathing.PID;

/**
 * Subsystem for a single-motor flywheel with PIDF velocity control.
 * Speed is read in encoder ticks per second from the encoder cable plugged into
 * the flywheel motor's own encoder port. Power each loop = F (kF * target + kS)
 * + PID on the velocity error. Gains are @Config so they show up in FTC Dashboard;
 * copy the tuned values back into this file, since Dashboard changes don't persist.
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
    public static double kP = 0.001, kI = 0.0, kD = 0.0;
    /** F (feedforward): power per tick/sec of target. Tune first, with kP/kI/kD at 0. */
    public static double kF = 0.00036;
    /** Static feedforward: constant power to overcome friction whenever the target isn't 0. */
    public static double kS = 0.0;

    // ---- dampening (active slowing down) -----------------------------------------
    /** Max reverse power the controller may apply when the flywheel is faster than the
     *  target, e.g. when the target drops. 0 = coast down only. Keep it small to spare the gearbox. */
    public static double MAX_BRAKE_POWER = 0.2;
    /** When the target is 0: true = motor brakes to a stop, false = flywheel coasts. */
    public static boolean BRAKE_WHEN_STOPPED = false;

    private final PID pid = new PID(kP, kI, kD);

    private double targetTicksPerSec = 0;
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

    public double getTicksPerSec() {
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
        motor.setZeroPowerBehavior(BRAKE_WHEN_STOPPED
                ? DcMotor.ZeroPowerBehavior.BRAKE
                : DcMotor.ZeroPowerBehavior.FLOAT);

        if (targetTicksPerSec <= 0) {
            pid.resetIntegral();
            lastPower = 0;
            motor.setPower(0);
            return;
        }

        pid.updatePID(kP, kI, kD);
        double power = kF * targetTicksPerSec + kS + pid.update(getError());
        // Allow a little reverse power to slow down faster when above target
        lastPower = Range.clip(power, -MAX_BRAKE_POWER, 1);
        motor.setPower(lastPower);
    }
}
