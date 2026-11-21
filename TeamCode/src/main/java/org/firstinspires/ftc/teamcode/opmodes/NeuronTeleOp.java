package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Gate;
import org.firstinspires.ftc.teamcode.subsystems.HeadingLight;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;


/**
 * Manual TeleOp on top of Neuron Pathing. Drive with the left stick's y/x and
 * rotate with the right stick's x -- robot-centric, no field-oriented math, so
 * it's a good first sanity check that your motors/directions are wired
 * correctly before layering in autonomous paths.
 *
 * Demonstrates the subsystem pattern using Gate (1 servo) and Intake (1 motor),
 * constructed alongside drive/localizer and updated every loop.
 *
 * Hold cross to auto aim at the goal (AutoAim): it takes over turning and drives the
 * robot to its shooting spot (distance from the goal + y zone); the left stick still adds on top.
 * NeuronTeleOpRed / NeuronTeleOpBlue always aim at their own alliance's goal. Subclasses without a fixed alliance (WroomTeleOp) take it from the
 * last auto; during init, press square for Blue or circle for Red to change it.
 *
 * This is the shared base for the Red/Blue TeleOps and isn't listed on the Driver
 * Station itself (no @TeleOp annotation).
 */
public abstract class NeuronTeleOp extends LinearOpMode {
    /** Max turning power (fraction of full motor power). Normal drive power is capped
     *  by NeuronDrive.teleOpMaxSpeed. */
    public static double TURN_MAX_POWER = 0.6;

    /** Max drive power: NeuronDrive.teleOpMaxSpeed. WroomTeleOp overrides this;
     *  everything else it inherits from here. */
    protected double driveMaxPower() {
        return NeuronDrive.teleOpMaxSpeed;
    }

    /** Max turning power for the right stick. */
    protected double turnMaxPower() {
        return TURN_MAX_POWER;
    }

    /** Fixed alliance for auto aim: -1 Blue, +1 Red. null = take it from the last auto
     *  and allow changing it during init. NeuronTeleOpRed / NeuronTeleOpBlue override this. */
    protected Double fixedAlliance() {
        return null;
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
        ElapsedTime headingLightHoldTimer = new ElapsedTime();
        // Alliance (-1 Blue / +1 Red): fixed by the Red/Blue TeleOps, otherwise from the
        // last auto, or Blue if auto hasn't run since a restart
        Double fixedAlliance = fixedAlliance();
        AutoAim aim = new AutoAim(fixedAlliance != null ? fixedAlliance
                : (double) blackboard.getOrDefault(PlannerAuto.ALLIANCE_KEY, -1.0));

        // Calibrate the Pinpoint during init, before the first drive.update() reads the pose.
        // calibrate(true) resets the position to 0, so the start pose is set afterwards:
        // wherever auto (or the last TeleOp) left the robot, or (0, 0, 0) after a restart
        localizer.calibrate(true);
        Pose2D startPose = (Pose2D) blackboard.getOrDefault(PlannerAuto.POSE_KEY, new Pose2D(0, 0, 0));
        localizer.setPosition(startPose);

        // Init loop instead of waitForStart(), so the alliance can be changed before START
        while (opModeInInit()) {
            if (fixedAlliance == null) {
                if (gamepad1.squareWasPressed()) aim.setAlliance(-1);
                if (gamepad1.circleWasPressed()) aim.setAlliance(1);
            }
            telemetry.addData("start pose", "x %.1f, y %.1f, heading %.0f deg",
                    startPose.x, startPose.y, Math.toDegrees(startPose.h));
            telemetry.addData("alliance (auto aim)", aim.getAlliance() > 0 ? "RED" : "BLUE");
            if (fixedAlliance == null) {
                telemetry.addLine("Square = Blue, Circle = Red");
            }
            telemetry.update();
        }
        if (isStopRequested()) return;

        // Spin the flywheel up at the start and keep it at shot speed for the whole match
        flywheel.setTargetTicksPerSec(Shooter.SHOOT_TICKS_PER_SEC);

        boolean wasAiming = false;
        while (opModeIsActive()) {
            // Drive and turn power caps come from driveMaxPower()/turnMaxPower().
            // The turn stick is scaled by turn cap / drive cap, since drive() already
            // multiplies everything by the drive cap
            boolean cross = gamepad1.cross;
            double driveMax = driveMaxPower();
            double turnMax = turnMaxPower();
            double turn;
            if (cross) {
                // Auto aim while cross is held. AutoAim is CCW-positive and gives the real
                // motor power; drive() is clockwise-positive and multiplies by driveMax
                if (!wasAiming) aim.reset();
                turn = -aim.update(drive.pose) / driveMax;
            } else {
                aim.updateTarget(drive.pose);
                turn = gamepad1.right_stick_x * Math.min(1, turnMax / driveMax);
            }
            wasAiming = cross;
            // While aiming, AutoAim also drives toward/away from the goal to its shooting
            // distance. Its powers are real motor powers, so divide by driveMax like the turn;
            // drive() strafes right-positive, AutoAim gives strafe left
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            if (cross) {
                forward += aim.getForwardPower() / driveMax;
                strafe -= aim.getStrafeLeftPower() / driveMax;
            }
            drive.drive(forward, strafe, turn, driveMax);
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
            blackboard.put(PlannerAuto.POSE_KEY, drive.pose);

            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.h));
            telemetry.addData("gate", gate.state);
            telemetry.addData("intake", intake.state);
            telemetry.addData("shooter", shooter.state);
            telemetry.addData("flywheel speed", flywheel.getTicksPerSec());
            telemetry.addData("auto aim", cross ? (aim.isAimed() ? "AIMED" : "aiming") : "off");
            telemetry.addData("goal distance (in)", "%.1f (target %.0f +- %.1f)", aim.getDistanceIn(),
                    AutoAim.AIM_DISTANCE_IN, AutoAim.AIM_DISTANCE_TOLERANCE_IN);
            telemetry.addData("shooting zone y", "%.0f to %.0f (%s)", aim.zoneYMin(), aim.zoneYMax(),
                    aim.isInZone() ? "in zone" : "outside");
            telemetry.addData("goal", "x %.1f, y %.1f (%s)", aim.getGoalX(), aim.getGoalY(),
                    aim.getAlliance() > 0 ? "RED" : "BLUE");
            telemetry.addData("heading light", headingLight.state);
            telemetry.addData("heading light angle (deg)", headingLight.getServoAngleDeg());
            telemetry.update();
        }
    }
}
