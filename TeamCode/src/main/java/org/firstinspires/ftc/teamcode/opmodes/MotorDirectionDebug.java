package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;

import java.util.ArrayList;
import java.util.List;

/**
 * Motor direction debugger for every motor on Box of Hopes and Dreams. Hold a button to
 * spin ONE motor at a time (gamepad1). Put the robot on blocks first.
 *
 * Drive motors (right-hand face buttons):
 *   Cross = frontLeft   Circle = frontRight   Square = backLeft   Triangle = backRight
 * Other motors (dpad):
 *   Up = intake   Right = flywheel   Down = mainArm   (left is unused)
 *
 * Hold L2 (left trigger) while pressing a button to spin that motor backwards.
 * R1 / L1  = raise / lower test power (0.05 steps). mainArm is capped at ARM_MAX_POWER.
 * Options  = toggle CODE directions (what TeleOp/auto use) vs RAW (all FORWARD).
 *
 * In CODE mode with positive power, each motor should:
 *   drive wheels -> roll the robot FORWARD
 *   intake       -> pull game pieces IN
 *   flywheel     -> spin the shooting direction
 *   mainArm      -> raise the arm (UP)
 * and its encoder should count UP. If a motor spins the wrong way, flip its direction in
 * the code (NeuronDrive, Intake, Flywheel.REVERSED, MainArm.REVERSED). If the encoder
 * counts down while the motor goes the right way, the encoder is reversed relative to it
 * (for the arm, set MainArm.ENCODER_REVERSED).
 * The arm's encoder is plugged into backLeft's encoder port, so the mainArm line reads it
 * from there, and the backLeft line's enc/vel show the ARM, not the backLeft wheel.
 * If the wrong motor spins, fix the names/ports in the robot configuration.
 */
@Config
@TeleOp(name = "Motor Direction Debug", group = "Tuning")
public class MotorDirectionDebug extends LinearOpMode {

    public static double POWER_STEP = 0.05;
    public static double MIN_POWER = 0.10;
    public static double MAX_POWER = 1.00;
    /** The arm swings into hard stops, so it never gets more than this, whatever the test power. */
    public static double ARM_MAX_POWER = 0.25;

    /** One motor under test: config name, label, the direction the code uses, and a power cap. */
    private static class TestMotor {
        final String name;
        final String label;
        final DcMotorSimple.Direction codeDirection;
        final double maxPower;
        DcMotorEx motor; // null if it's missing from the configuration
        /** Motor whose encoder port this motor's encoder is plugged into (itself by default). */
        TestMotor encoderFrom = this;
        /** -1 to flip the encoder reading (e.g. MainArm.ENCODER_REVERSED). */
        double encoderSign = 1;

        TestMotor(String name, String label, DcMotorSimple.Direction codeDirection, double maxPower) {
            this.name = name;
            this.label = label;
            this.codeDirection = codeDirection;
            this.maxPower = maxPower;
        }
    }

    private final List<TestMotor> motors = new ArrayList<>();
    private double testPower = 0.30;
    private boolean useCodeDirections = true;

    @Override
    public void runOpMode() {
        DcMotorSimple.Direction fwd = DcMotorSimple.Direction.FORWARD;
        DcMotorSimple.Direction rev = DcMotorSimple.Direction.REVERSE;

        // Directions match what the drive and subsystems set in their constructors
        TestMotor frontLeft  = add("frontLeft",  "frontLeft  (Cross)",    fwd, 1.0);
        TestMotor frontRight = add("frontRight", "frontRight (Circle)",   rev, 1.0);
        TestMotor backLeft   = add("backLeft",   "backLeft   (Square)",   fwd, 1.0);
        TestMotor backRight  = add("backRight",  "backRight  (Triangle)", rev, 1.0);
        TestMotor intake     = add("intake",     "intake     (Dpad up)",  fwd, 1.0);
        TestMotor flywheel   = add("flywheel",   "flywheel   (Dpad right)", Flywheel.REVERSED ? rev : fwd, 1.0);
        TestMotor mainArm    = add("mainArm",    "mainArm    (Dpad down)", MainArm.REVERSED ? rev : fwd, ARM_MAX_POWER);
        // The arm's encoder is plugged into backLeft's encoder port; read it the way MainArm does
        mainArm.encoderFrom = backLeft;
        mainArm.encoderSign = MainArm.ENCODER_REVERSED ? -1 : 1;

        for (TestMotor t : motors) {
            try {
                t.motor = hardwareMap.get(DcMotorEx.class, t.name);
                t.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                t.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                t.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } catch (IllegalArgumentException e) {
                t.motor = null;
            }
        }
        applyDirections();

        telemetry.addLine("Motor Direction Debug - put the robot on blocks!");
        telemetry.addLine("Cross=FL  Circle=FR  Square=BL  Triangle=BR");
        telemetry.addLine("Dpad: up=intake  right=flywheel  down=mainArm");
        telemetry.addLine("Hold L2 = backwards, R1/L1 = power, Options = CODE/RAW");
        for (TestMotor t : motors) {
            if (t.motor == null) telemetry.addData("MISSING from config", t.name);
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) {
                testPower = Range.clip(testPower + POWER_STEP, MIN_POWER, MAX_POWER);
            }
            if (gamepad1.leftBumperWasPressed()) {
                testPower = Range.clip(testPower - POWER_STEP, MIN_POWER, MAX_POWER);
            }
            if (gamepad1.optionsWasPressed()) {
                useCodeDirections = !useCodeDirections;
                stopAll();
                applyDirections();
            }

            // Only one motor at a time; if several buttons are held, the first one listed wins
            TestMotor active = null;
            if (gamepad1.cross) active = frontLeft;
            else if (gamepad1.circle) active = frontRight;
            else if (gamepad1.square) active = backLeft;
            else if (gamepad1.triangle) active = backRight;
            else if (gamepad1.dpad_up) active = intake;
            else if (gamepad1.dpad_right) active = flywheel;
            else if (gamepad1.dpad_down) active = mainArm;

            double sign = gamepad1.left_trigger > 0.5 ? -1 : 1;
            for (TestMotor t : motors) {
                if (t.motor == null) continue;
                t.motor.setPower(t == active ? sign * Math.min(testPower, t.maxPower) : 0);
            }

            telemetry.addData("Active", active == null ? "none"
                    : active.name + (sign < 0 ? " (BACKWARDS)" : ""));
            telemetry.addData("Test power", "%.2f  (arm max %.2f)", testPower, ARM_MAX_POWER);
            telemetry.addData("Directions", useCodeDirections
                    ? "CODE (what TeleOp/auto use)" : "RAW (all FORWARD)");
            telemetry.addLine();
            for (TestMotor t : motors) {
                if (t.motor == null) {
                    telemetry.addData(t.label, "MISSING from config");
                } else if (t.encoderFrom != t) {
                    // Encoder lives on another motor's port: undo that motor's direction, apply our sign
                    DcMotorEx enc = t.encoderFrom.motor;
                    if (enc == null) {
                        telemetry.addData(t.label, "dir=%s  pwr=%.2f  enc: %s MISSING",
                                t.motor.getDirection(), t.motor.getPower(), t.encoderFrom.name);
                    } else {
                        double dirSign = enc.getDirection() == DcMotorSimple.Direction.REVERSE ? -1 : 1;
                        telemetry.addData(t.label, "dir=%s  pwr=%.2f  enc=%.0f  vel=%.0f  (enc on %s port)",
                                t.motor.getDirection(), t.motor.getPower(),
                                t.encoderSign * dirSign * enc.getCurrentPosition(),
                                t.encoderSign * dirSign * enc.getVelocity(), t.encoderFrom.name);
                    }
                } else {
                    telemetry.addData(t.label, "dir=%s  pwr=%.2f  enc=%d  vel=%.0f",
                            t.motor.getDirection(), t.motor.getPower(),
                            t.motor.getCurrentPosition(), t.motor.getVelocity());
                }
            }
            telemetry.update();
        }

        stopAll();
    }

    private TestMotor add(String name, String label, DcMotorSimple.Direction dir, double maxPower) {
        TestMotor t = new TestMotor(name, label, dir, maxPower);
        motors.add(t);
        return t;
    }

    private void applyDirections() {
        for (TestMotor t : motors) {
            if (t.motor == null) continue;
            t.motor.setDirection(useCodeDirections ? t.codeDirection : DcMotorSimple.Direction.FORWARD);
        }
    }

    private void stopAll() {
        for (TestMotor t : motors) {
            if (t.motor != null) t.motor.setPower(0);
        }
    }
}
