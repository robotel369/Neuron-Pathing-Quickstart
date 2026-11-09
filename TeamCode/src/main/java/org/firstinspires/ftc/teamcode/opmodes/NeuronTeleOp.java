package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.ArmPivotMacro;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;


/**
 * Box of Hopes and Dreams TeleOp, on top of Neuron Pathing. Robot-centric drive.
 *
 * Gamepad1 (driver):
 *   Left stick = drive (forward/back, strafe), right stick X = turn, hold Cross = boost
 *   R2 / L2    = intake in / out; power equals how far the trigger is pressed.
 *                While intaking, the flywheel spins inward at FLYWHEEL_INTAKE_POWER.
 *   Circle     = shoot macro (or cancel a shot that's running)
 *   R1         = toggle gate
 *   Dpad up    = UP macro:   pivot TRANSITION, then arm up + pivot SHOOT_FRONT together
 *   Dpad down  = DOWN macro: arm down, then pivot TRANSITION, then pivot BOTTOM
 *
 * Gamepad2 (operator, manual overrides; any of these cancels the up/down macro):
 *   Left stick Y = move the main arm (goal moves at ARM_MANUAL_TICKS_PER_SEC at full stick)
 *   Dpad down = pivot BOTTOM, dpad left = TRANSITION, dpad up = SHOOT_FRONT, dpad right = SHOOT_BACK
 *
 * The main arm must rest in its stowed position when the OpMode inits (encoder zeroes then).
 */
@TeleOp(name = "Neuron TeleOp (Basic)")
public class NeuronTeleOp extends LinearOpMode {
    /** Max turning power (fraction of full motor power). Normal drive power is capped
     *  by NeuronDrive.teleOpMaxSpeed. */
    public static double TURN_MAX_POWER = 0.6;
    /** Max drive and turning power while cross is held. */
    public static double BOOST_DRIVE_MAX_POWER = 1.0;
    public static double BOOST_TURN_MAX_POWER = 0.8;
    /** Flywheel power (backwards, pulling inward) while the driver intakes with R2. 0 = off. */
    public static double FLYWHEEL_INTAKE_POWER = 0.4;
    /** Main arm goal speed at full gamepad2 left stick, in encoder ticks per second. */
    public static double ARM_MANUAL_TICKS_PER_SEC = 400;

    /** Max drive power: NeuronDrive.teleOpMaxSpeed normally, BOOST_DRIVE_MAX_POWER while
     *  cross is held. WroomTeleOp overrides this; everything else it inherits from here. */
    protected double driveMaxPower(boolean crossHeld) {
        return crossHeld ? BOOST_DRIVE_MAX_POWER : NeuronDrive.teleOpMaxSpeed;
    }

    /** Max turning power: TURN_MAX_POWER normally, BOOST_TURN_MAX_POWER while cross is held. */
    protected double turnMaxPower(boolean crossHeld) {
        return crossHeld ? BOOST_TURN_MAX_POWER : TURN_MAX_POWER;
    }

    @Override
    public void runOpMode() {
        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/144.0, /*yOffsetMm=*/-144.0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/405.0 / 25.4);
        Gate gate = new Gate(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Shooter shooter = new Shooter(flywheel, intake, gate);
        MainArm arm = new MainArm(hardwareMap);
        ShooterPivot pivot = new ShooterPivot(hardwareMap);
        ArmPivotMacro armMacro = new ArmPivotMacro(arm, pivot);
        ElapsedTime loopTimer = new ElapsedTime();
        // True while TeleOp (not the shoot macro) is spinning the flywheel inward for intaking
        boolean flywheelIntaking = false;

        // Calibrate the Pinpoint during init, before the first drive.update() reads the pose.
        // calibrate(true) resets the position to 0, so the start pose is set afterwards:
        // wherever auto (or the last TeleOp) left the robot, or (0, 0, 0) after a restart
        localizer.calibrate(true);
        Pose2D startPose = (Pose2D) blackboard.getOrDefault(PlannerAuto.POSE_KEY, new Pose2D(0, 0, 0));
        localizer.setPosition(startPose);
        telemetry.addData("start pose", "x %.1f, y %.1f, heading %.0f deg",
                startPose.x, startPose.y, Math.toDegrees(startPose.h));
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Drive and turn power caps come from driveMaxPower()/turnMaxPower() (cross held or not).
            // The turn stick is scaled by turn cap / drive cap, since drive() already
            // multiplies everything by the drive cap
            boolean cross = gamepad1.cross;
            double driveMax = driveMaxPower(cross);
            double turnMax = turnMaxPower(cross);
            double turn = gamepad1.right_stick_x * Math.min(1, turnMax / driveMax);
            drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, turn, driveMax);
            drive.update();

            // Shoot macro on circle edge press: starts a shot, or cancels one that's running
            if (gamepad1.circleWasPressed()) {
                if (shooter.isBusy()) {
                    shooter.abort();
                } else {
                    flywheelIntaking = false; // the shot takes over the flywheel
                    shooter.start();
                }
            }
            shooter.update();

            // Manual gate/intake controls are ignored while the macro runs, so they
            // can't close the gate or override the intake's feed pulses mid-shot
            if (!shooter.isBusy()) {
                // Intake power follows how far the triggers are pressed: R2 intakes, L2 outtakes.
                // Small presses below 0.05 are ignored.
                double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
                intake.setPower(Math.abs(intakePower) > 0.05 ? intakePower : 0);

                // While intaking, spin the flywheel inward (opposite of shooting) so pieces
                // are pulled in. Only stop it again if TeleOp started it, so this never
                // cuts power the shoot macro left on.
                if (intakePower > 0.05 && FLYWHEEL_INTAKE_POWER > 0) {
                    flywheel.setOpenLoopPower(-FLYWHEEL_INTAKE_POWER);
                    flywheelIntaking = true;
                } else if (flywheelIntaking) {
                    flywheel.stop();
                    flywheelIntaking = false;
                }

                // Gate toggle on R1 (right bumper) edge press: each press flips open/closed
                if (gamepad1.rightBumperWasPressed()) {
                    gate.toggle();
                }
            }
            gate.update();
            intake.update();
            flywheel.update();

            // Arm + pivot up/down macro on gamepad1's dpad
            if (gamepad1.dpadUpWasPressed()) armMacro.up();
            if (gamepad1.dpadDownWasPressed()) armMacro.down();
            armMacro.update();

            // Manual overrides on gamepad2; any of them cancels the macro so they don't fight
            // Main arm: left stick moves the goal; the arm's PID follows it smoothly
            double armStick = -gamepad2.left_stick_y;
            if (Math.abs(armStick) > 0.05) {
                armMacro.cancel();
                arm.adjustTarget(armStick * ARM_MANUAL_TICKS_PER_SEC * dt);
            }
            // Shooter pivot presets on the dpad
            if (gamepad2.dpadDownWasPressed()) { armMacro.cancel(); pivot.bottom(); }
            if (gamepad2.dpadLeftWasPressed()) { armMacro.cancel(); pivot.transition(); }
            if (gamepad2.dpadUpWasPressed()) { armMacro.cancel(); pivot.shootFront(); }
            if (gamepad2.dpadRightWasPressed()) { armMacro.cancel(); pivot.shootBack(); }
            arm.update();
            pivot.update();

            blackboard.put(PlannerAuto.POSE_KEY, drive.pose);

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("gate", gate.state);
            telemetry.addData("intake", intake.state);
            telemetry.addData("shooter", shooter.state);
            telemetry.addData("flywheel", "power %.2f%s", flywheel.getPower(),
                    flywheel.isOpenLoop() ? " (no encoder, fixed power)"
                            : String.format("  speed %.0f", flywheel.getTicksPerSec()));
            telemetry.addData("arm macro", armMacro.state);
            telemetry.addData("arm", "%s  pos %.0f / goal %.0f", arm.state, arm.getPosition(), arm.getGoal());
            telemetry.addData("pivot", "%s (%.3f)", pivot.pose, pivot.getPosition());
            telemetry.update();
        }
    }
}
