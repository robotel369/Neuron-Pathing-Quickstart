package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.HeadingLight;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MainArm;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPivot;


/**
 * Manual TeleOp on top of Neuron Pathing. Drive with the left stick's y/x and
 * rotate with the right stick's x -- robot-centric, no field-oriented math, so
 * it's a good first sanity check that your motors/directions are wired
 * correctly before layering in autonomous paths.
 *
 * Demonstrates the subsystem pattern using Gate (1 servo) and Intake (1 motor),
 * constructed alongside drive/localizer and updated every loop.
 *
 * Box of Hopes and Dreams additions, on gamepad2 (operator):
 *   Left stick Y = move the main arm (goal moves at ARM_MANUAL_TICKS_PER_SEC at full stick)
 *   Dpad down = pivot BOTTOM, dpad left = TRANSITION, dpad up = SHOOT_FRONT, dpad right = SHOOT_BACK
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
        HeadingLight headingLight = new HeadingLight(hardwareMap);
        MainArm arm = new MainArm(hardwareMap);
        ShooterPivot pivot = new ShooterPivot(hardwareMap);
        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime headingLightHoldTimer = new ElapsedTime();

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

                // Gate toggle on R1 (right bumper) edge press: each press flips open/closed
                if (gamepad1.rightBumperWasPressed()) {
                    gate.toggle();
                }
            }
            gate.update();
            intake.update();
            flywheel.update();
            // Heading light: tap dpad right to turn the servo and LED fully off,
            // hold it for 1.5 s to turn them back on
            if (gamepad1.dpadRightWasPressed()) {
                headingLight.disable();
                headingLightHoldTimer.reset();
            }
            if (gamepad1.dpad_right && !headingLight.isEnabled()
                    && headingLightHoldTimer.milliseconds() >= 1500) {
                headingLight.enable();
            }
            headingLight.update(drive.pose.h);

            // Main arm: gamepad2 left stick moves the goal; the arm's PID follows it smoothly
            double armStick = -gamepad2.left_stick_y;
            if (Math.abs(armStick) > 0.05) {
                arm.adjustTarget(armStick * ARM_MANUAL_TICKS_PER_SEC * dt);
            }
            arm.update();

            // Shooter pivot presets on gamepad2's dpad
            if (gamepad2.dpadDownWasPressed()) pivot.bottom();
            if (gamepad2.dpadLeftWasPressed()) pivot.transition();
            if (gamepad2.dpadUpWasPressed()) pivot.shootFront();
            if (gamepad2.dpadRightWasPressed()) pivot.shootBack();
            pivot.update();

            blackboard.put(PlannerAuto.POSE_KEY, drive.pose);

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("gate", gate.state);
            telemetry.addData("intake", intake.state);
            telemetry.addData("shooter", shooter.state);
            telemetry.addData("flywheel speed", flywheel.getTicksPerSec());
            telemetry.addData("heading light", headingLight.state);
            telemetry.addData("arm", "%s  pos %.0f / goal %.0f", arm.state, arm.getPosition(), arm.getGoal());
            telemetry.addData("pivot", "%s (%.2f)", pivot.pose, pivot.getPosition());
            telemetry.addData("heading light angle (deg)", headingLight.getServoAngleDeg());
            telemetry.update();
        }
    }
}
