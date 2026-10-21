package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for controlling a gate servo.
 * Follows Neuron Pathing subsystem conventions.
 */
public class Gate {
    public enum State {
        OPEN,
        CLOSED
    }

    private final Servo servo;
    public State state = State.CLOSED;

    public static double OPEN_POSITION = 0.5;
    public static double CLOSED_POSITION = 0.0;

    public Gate(HardwareMap hwMap, String servoName) {
        servo = hwMap.get(Servo.class, servoName);
        close();
    }

    public Gate(HardwareMap hwMap) {
        this(hwMap, "gate");
    }

    public void open() {
        servo.setPosition(OPEN_POSITION);
        state = State.OPEN;
    }

    public void close() {
        servo.setPosition(CLOSED_POSITION);
        state = State.CLOSED;
    }

    public void toggle() {
        if (state == State.OPEN) {
            close();
        } else {
            open();
        }
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public double getPosition() {
        return servo.getPosition();
    }

    /**
     * Called continuously in TeleOp and Autonomous loops.
     */
    public void update() {
        // Reserved for state machine logic or telemetry updates
    }
}
