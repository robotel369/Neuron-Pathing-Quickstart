package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Subsystem for a goBILDA Super Speed servo that keeps a mounted goBILDA LED pointing at
 * one field heading while the robot drives and turns. Each loop the servo turns opposite
 * to the robot's rotation, using the Pinpoint heading from drive.pose.h.
 *
 * The servo only turns SERVO_RANGE_DEG (360) from end to end, so it can't keep turning the
 * same way forever. When the heading moves past either end, it holds at that end for
 * WRAP_HYSTERESIS_DEG more, then wraps: it swings 360 deg back the other way and points the
 * same direction from the other side. The LED is at full power all the time, except while a
 * wrap is happening.
 *
 * Servo position 0.5 (the middle of its travel) is assumed to point straight out the front
 * of the robot; use MOUNT_OFFSET_DEG if it doesn't. Values are @Config so they show up in
 * FTC Dashboard; copy tuned values back into this file, since Dashboard changes don't persist.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class HeadingLight {
    public enum State {
        TRACKING,
        WRAPPING,
        DISABLED
    }

    private final ServoImplEx servo;
    private final CRServoImplEx led;
    public State state = State.TRACKING;

    /** Field heading the LED should point at, in degrees. Same convention as drive.pose.h:
     *  0 = the direction the robot faced when the pose was set, counter-clockwise positive. */
    public static double TARGET_HEADING_DEG = 180;
    /** Total travel of the servo from position 0 to position 1. */
    public static double SERVO_RANGE_DEG = 360;
    /** How far the heading can go past the end of travel (with the servo held at its end)
     *  before it wraps. Stops it wrapping back and forth when the heading sits right at the end. */
    public static double WRAP_HYSTERESIS_DEG = 10;
    /** Servo angle (relative to robot forward) at position 0.5. Adjust if the LED isn't
     *  pointing straight out the front of the robot at the middle of the servo's travel. */
    public static double MOUNT_OFFSET_DEG = 0;
    /** Flip if the LED turns the wrong way (clockwise when the code asks for counter-clockwise). */
    public static boolean SERVO_REVERSED = true;
    /** How long the LED stays off during a wrap: roughly how long the servo takes to swing 360 deg. */
    public static double WRAP_LED_OFF_MS = 400;

    /** LED output when it's on / off. The LED is driven as a CR servo, so power 1.0 is full
     *  brightness. */
    public static double LED_ON_POWER = 1.0;
    public static double LED_OFF_POWER = 0.0;

    private final ElapsedTime wrapTimer = new ElapsedTime();
    /** Current servo angle command relative to robot forward. Can be up to WRAP_HYSTERESIS_DEG
     *  past either end of travel; the servo position is clipped to its end there. */
    private double servoAngleDeg = 0;

    public HeadingLight(HardwareMap hwMap, String servoName, String ledName) {
        servo = hwMap.get(ServoImplEx.class, servoName);
        led = hwMap.get(CRServoImplEx.class, ledName);
        setServoAngle(0);
        led.setPower(LED_ON_POWER);
    }

    public HeadingLight(HardwareMap hwMap) {
        this(hwMap, "lightServo", "light");
    }

    public void setTargetHeadingDeg(double headingDeg) {
        TARGET_HEADING_DEG = headingDeg;
    }

    public double getServoAngleDeg() {
        return servoAngleDeg;
    }

    public boolean isWrapping() {
        return state == State.WRAPPING;
    }

    public boolean isEnabled() {
        return state != State.DISABLED;
    }

    /** Cuts the servo's PWM signal so it goes limp (only this port is affected), and sets
     *  the LED to LED_OFF_POWER. The LED keeps its signal, because with no signal at all it
     *  can stay lit. Both stay off until enable() is called. */
    public void disable() {
        servo.setPwmDisable();
        led.setPower(LED_OFF_POWER);
        state = State.DISABLED;
    }

    /** Turns the servo and LED back on; the next update() points the LED at the heading again. */
    public void enable() {
        if (isEnabled()) return;
        servo.setPwmEnable();
        state = State.TRACKING;
    }

    /**
     * Called continuously in TeleOp and Autonomous loops, after drive.update().
     * Does nothing while disabled.
     * @param robotHeadingRad the robot's current heading, drive.pose.h
     */
    public void update(double robotHeadingRad) {
        if (state == State.DISABLED) return;

        // Angle the LED has to point relative to the robot, -180 to 180
        double relativeDeg = wrapDeg(TARGET_HEADING_DEG - Math.toDegrees(robotHeadingRad) - MOUNT_OFFSET_DEG);

        // Of all the servo angles that point that way (relativeDeg + n * 360), take the
        // one nearest where the servo is now, so it keeps following smoothly
        double desiredDeg = relativeDeg + 360 * Math.round((servoAngleDeg - relativeDeg) / 360);

        // Too far past the end of travel: wrap 360 back the other way, with the LED off while it swings
        double limitDeg = SERVO_RANGE_DEG / 2 + WRAP_HYSTERESIS_DEG;
        if (Math.abs(desiredDeg) > limitDeg) {
            desiredDeg -= 360 * Math.signum(desiredDeg);
            state = State.WRAPPING;
            wrapTimer.reset();
        }
        setServoAngle(desiredDeg);

        if (state == State.WRAPPING && wrapTimer.milliseconds() >= WRAP_LED_OFF_MS) {
            state = State.TRACKING;
        }
        led.setPower(state == State.WRAPPING ? LED_OFF_POWER : LED_ON_POWER);
    }

    private void setServoAngle(double angleDeg) {
        servoAngleDeg = angleDeg;
        double direction = SERVO_REVERSED ? -1 : 1;
        servo.setPosition(Range.clip(0.5 + direction * angleDeg / SERVO_RANGE_DEG, 0, 1));
    }

    /** Wraps an angle in degrees to -180..180. */
    private static double wrapDeg(double angleDeg) {
        angleDeg %= 360;
        if (angleDeg > 180) angleDeg -= 360;
        if (angleDeg <= -180) angleDeg += 360;
        return angleDeg;
    }
}
