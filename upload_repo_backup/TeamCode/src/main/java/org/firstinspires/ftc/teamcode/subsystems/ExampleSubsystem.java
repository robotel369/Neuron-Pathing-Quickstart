package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A template for creating your own robot mechanisms (intake, arm, etc.).
 * Follows the "Subsystems" pattern recommended by Neuron Pathing.
 */
public class ExampleSubsystem {
    
    // Define your states here (Architecture for Actions)
    public enum State {
        IDLE,
        ACTION_START,
        ACTION_WAIT,
        ACTION_FINISHED
    }

    private State currentState = State.IDLE;
    private final ElapsedTime timer = new ElapsedTime();

    public ExampleSubsystem(HardwareMap hwMap) {
        // Initialize your hardware here (e.g. motor = hwMap.get(DcMotor.class, "name"))
    }

    /** Initiates a non-blocking sequence. */
    public void startAction() {
        if (currentState == State.IDLE) {
            currentState = State.ACTION_START;
            timer.reset();
        }
    }

    /** 
     * Call this every loop from TeleOp or driveToTarget() in Auto.
     * This handles the state machine logic without blocking the drivetrain.
     */
    public void update() {
        switch (currentState) {
            case IDLE:
                break;
            case ACTION_START:
                // Start a motor or move a servo
                currentState = State.ACTION_WAIT;
                break;
            case ACTION_WAIT:
                if (timer.milliseconds() > 500) { // Example delay
                    currentState = State.ACTION_FINISHED;
                }
                break;
            case ACTION_FINISHED:
                currentState = State.IDLE;
                break;
        }
    }

    public boolean isBusy() {
        return currentState != State.IDLE;
    }
}
