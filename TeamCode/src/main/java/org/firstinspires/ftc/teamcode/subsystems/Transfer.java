package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Subsystem for a transfer driven by three continuous rotation servos.
 * All three servos are always commanded together with the same power, so the
 * transfer moves as one system and never fights itself.
 * Follows Neuron Pathing subsystem conventions.
 */
public class Transfer {
    public enum State {
        FEEDING,
        REVERSING,
        STOPPED
    }

    private final CRServo[] servos;
    public State state = State.STOPPED;

    private static final double FEED_POWER = 1.0;
    private static final double REVERSE_POWER = -1.0;

    // Flip any of these if that servo is mounted mirrored and spins against the others
    public static boolean TRANSFER1_REVERSED = false;
    public static boolean TRANSFER2_REVERSED = false;
    public static boolean TRANSFER3_REVERSED = false;

    public Transfer(HardwareMap hwMap, String name1, String name2, String name3) {
        servos = new CRServo[] {
                hwMap.get(CRServo.class, name1),
                hwMap.get(CRServo.class, name2),
                hwMap.get(CRServo.class, name3)
        };
        boolean[] reversed = {TRANSFER1_REVERSED, TRANSFER2_REVERSED, TRANSFER3_REVERSED};
        for (int i = 0; i < servos.length; i++) {
            servos[i].setDirection(reversed[i]
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
        }
        stop();
    }

    public Transfer(HardwareMap hwMap) {
        this(hwMap, "transfer1", "transfer2", "transfer3");
    }

    public void feed() {
        setPower(FEED_POWER);
    }

    public void reverse() {
        setPower(REVERSE_POWER);
    }

    public void stop() {
        setPower(0.0);
    }

    public void setPower(double power) {
        for (CRServo servo : servos) {
            servo.setPower(power);
        }
        if (power > 0) {
            state = State.FEEDING;
        } else if (power < 0) {
            state = State.REVERSING;
        } else {
            state = State.STOPPED;
        }
    }

    public double getPower() {
        return servos[0].getPower();
    }

    /**
     * Called continuously in TeleOp and Autonomous loops.
     */
    public void update() {
        // Reserved for state machine logic or telemetry updates
    }
}
