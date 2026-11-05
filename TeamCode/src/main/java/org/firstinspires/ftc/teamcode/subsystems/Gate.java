package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Subsystem for controlling a gate servo.
 * Positions are @Config so they can be adjusted live in FTC Dashboard (or with the
 * "Gate Position Tuning" OpMode); copy the final values back into this file.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class Gate {
    public enum State {
        OPEN,
        CLOSED
    }

    private final Servo servo;
    public State state = State.CLOSED;

    // Box of Hopes and Dreams values. Tune on the robot.
    public static double OPEN_POSITION = 0.6;
    public static double CLOSED_POSITION = 0.8;

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
