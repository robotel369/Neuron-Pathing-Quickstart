package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

/**
 * Drive motor assignment + direction debug for Neuron Pathing.
 *
 * Hold a face button to spin ONE drive motor at a time:
 *   A = frontLeft    B = frontRight
 *   X = backLeft     Y = backRight
 *
 * What to check (robot on blocks, wheels off the ground):
 *   1. ASSIGNMENT: the wheel that spins is the one named on telemetry.
 *      If not, fix the names in your robot configuration (or the NAMES below).
 *   2. DIRECTION: with the default (standard mecanum) directions applied, every
 *      wheel should spin so the robot would roll FORWARD.
 *      The Neuron docs assume frontLeft/backLeft are the reversed motors.
 *   3. ENCODERS: the encoder count shown should go UP when the wheel spins forward.
 *
 * Controls:
 *   Left bumper  : toggle "Neuron default directions" (left side reversed) vs RAW (all FORWARD)
 *   D-pad up/down: raise/lower test power (0.05 steps, 0.10 to 1.00)
 *
 * Tip: run once in RAW mode to see each motor's natural direction, then in
 * DEFAULT mode to confirm the robot-forward result.
 */
@TeleOp(name = "Neuron Motor Debug", group = "Neuron")
public class NeuronMotorDebug extends LinearOpMode {

    // Hardware config names (the defaults the Neuron Pathing docs use)
    private static final String FRONT_LEFT_NAME  = "frontLeft";
    private static final String FRONT_RIGHT_NAME = "frontRight";
    private static final String BACK_LEFT_NAME   = "backLeft";
    private static final String BACK_RIGHT_NAME  = "backRight";

    private static final double POWER_STEP = 0.05;
    private static final double MIN_POWER = 0.10;
    private static final double MAX_POWER = 1.00;

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;

    private double testPower = 0.30;
    private boolean useNeuronDefaults = true;

    // edge detection for toggles
    private boolean lastLeftBumper, lastDpadUp, lastDpadDown;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_NAME);
        backLeft   = hardwareMap.get(DcMotorEx.class, BACK_LEFT_NAME);
        backRight  = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_NAME);

        for (DcMotorEx m : new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        applyDirections();

        telemetry.addLine("Neuron Motor Debug ready");
        telemetry.addLine("A=FL  B=FR  X=BL  Y=BR (hold)");
        telemetry.addLine("LB = toggle directions, DPad up/down = power");
        telemetry.addLine("Put the robot on blocks first!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleToggles();

            // Only one motor runs at a time; if several buttons are held,
            // priority is A, B, X, Y so wheels never fight each other.
            double flP = 0, frP = 0, blP = 0, brP = 0;
            String active = "none";

            if (gamepad1.a) {
                flP = testPower;
                active = "frontLeft";
            } else if (gamepad1.b) {
                frP = testPower;
                active = "frontRight";
            } else if (gamepad1.x) {
                blP = testPower;
                active = "backLeft";
            } else if (gamepad1.y) {
                brP = testPower;
                active = "backRight";
            }

            frontLeft.setPower(flP);
            frontRight.setPower(frP);
            backLeft.setPower(blP);
            backRight.setPower(brP);

            telemetry.addData("Active motor", active);
            telemetry.addData("Test power", "%.2f", testPower);
            telemetry.addData("Directions", useNeuronDefaults
                    ? "NEURON DEFAULT (FL, BL reversed)"
                    : "RAW (all FORWARD)");
            telemetry.addLine();
            addMotorLine("frontLeft  (A)", frontLeft);
            addMotorLine("frontRight (B)", frontRight);
            addMotorLine("backLeft   (X)", backLeft);
            addMotorLine("backRight  (Y)", backRight);
            telemetry.update();
        }

        stopAll();
    }

    private void handleToggles() {
        boolean lb = gamepad1.left_bumper;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        if (lb && !lastLeftBumper) {
            useNeuronDefaults = !useNeuronDefaults;
            stopAll();
            applyDirections();
        }
        if (up && !lastDpadUp) {
            testPower = Range.clip(testPower + POWER_STEP, MIN_POWER, MAX_POWER);
        }
        if (down && !lastDpadDown) {
            testPower = Range.clip(testPower - POWER_STEP, MIN_POWER, MAX_POWER);
        }

        lastLeftBumper = lb;
        lastDpadUp = up;
        lastDpadDown = down;
    }

    private void applyDirections() {
        if (useNeuronDefaults) {
            // Same convention NeuronDrive's constructor uses for standard mecanum wiring
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void addMotorLine(String label, DcMotorEx m) {
        telemetry.addData(label, "dir=%s  power=%.2f  enc=%d",
                m.getDirection(), m.getPower(), m.getCurrentPosition());
    }

    private void stopAll() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}