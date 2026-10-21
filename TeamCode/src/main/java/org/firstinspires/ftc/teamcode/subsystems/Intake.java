package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for controlling a single-motor intake mechanism.
 * Follows Neuron Pathing subsystem conventions.
 */
public class Intake {
    public enum State {
        INTAKING,
        OUTTAKING,
        STOPPED
    }

    private final DcMotorEx motor;
    public State state = State.STOPPED;

    private static final double INTAKE_POWER = 1.0;
    private static final double OUTTAKE_POWER = -1.0;

    public Intake(HardwareMap hwMap, String motorName) {
        motor = hwMap.get(DcMotorEx.class, motorName);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stop();
    }

    public Intake(HardwareMap hwMap) {
        this(hwMap, "intake");
    }

    public void intake() {
        motor.setPower(INTAKE_POWER);
        state = State.INTAKING;
    }

    public void outtake() {
        motor.setPower(OUTTAKE_POWER);
        state = State.OUTTAKING;
    }

    public void stop() {
        motor.setPower(0.0);
        state = State.STOPPED;
    }

    public void setPower(double power) {
        motor.setPower(power);
        if (power > 0) {
            state = State.INTAKING;
        } else if (power < 0) {
            state = State.OUTTAKING;
        } else {
            state = State.STOPPED;
        }
    }

    public double getPower() {
        return motor.getPower();
    }

    /**
     * Called continuously in TeleOp and Autonomous loops.
     */
    public void update() {
        // Reserved for state machine logic, current monitoring, or telemetry
    }
}
