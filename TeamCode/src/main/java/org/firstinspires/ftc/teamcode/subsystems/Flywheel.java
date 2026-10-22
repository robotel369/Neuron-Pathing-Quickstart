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
 * Speed is read from the encoder cable plugged into the flywheel motor's own
 * encoder port. Power each loop = F (kF * target + kS) + PID on the velocity
 * error. Gains are @Config so they show up in FTC Dashboard; copy the
 * tuned values back into this file, since Dashboard changes don't persist.
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

    /** Encoder ticks per flywheel revolution. 28 for a bare goBILDA/REV motor encoder;
     *  multiply by any gear ratio between the motor and the flywheel. */
    public static double TICKS_PER_REV = 28;
    public static boolean REVERSED = false;

    // ---- tunable PIDF velocity gains (power units per RPM) -----------------------
    public static double kP = 0.0005, kI = 0.0, kD = 0.0;
    /** F (feedforward): power per RPM of target. Tune first, with kP/kI/kD at 0. */
    public static double kF = 0.00017;
    /** Static feedforward: constant power to overcome friction whenever the target isn't 0. */
    public static double kS = 0.0;

    private final PID pid = new PID(kP, kI, kD);

    private double targetRpm = 0;
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

    public void setTargetRpm(double rpm) {
        targetRpm = Math.max(0, rpm);
        state = targetRpm > 0 ? State.SPINNING : State.STOPPED;
    }

    public void stop() {
        setTargetRpm(0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getRpm() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getError() {
        return targetRpm - getRpm();
    }

    public double getPower() {
        return lastPower;
    }

    public boolean atSpeed(double toleranceRpm) {
        return targetRpm > 0 && Math.abs(getError()) < toleranceRpm;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops. Runs the velocity controller.
     */
    public void update() {
        if (targetRpm <= 0) {
            pid.resetIntegral();
            lastPower = 0;
            motor.setPower(0);
            return;
        }

        pid.updatePID(kP, kI, kD);
        double power = kF * targetRpm + kS + pid.update(getError());
        // Never drive the flywheel backwards to slow it down; let it coast instead
        lastPower = Range.clip(power, 0, 1);
        motor.setPower(lastPower);
    }
}
