package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.neuronpathing.NeuronDrive;
import org.firstinspires.ftc.teamcode.neuronpathing.NeuronLocalizer;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.AutoAim;

/**
 * Auto aim tuning OpMode with FTC Dashboard.
 *
 * Drive with the left stick and turn with the right stick as usual. Hold cross to auto
 * aim at the goal (or set AIM_WITHOUT_BUTTON to aim the whole time). Drive across x = 0
 * to check that the goal switches between the two hives.
 *
 * Connect to FTC Dashboard at http://192.168.43.1:8080/dash. Gains, goal positions and
 * AIM_OFFSET_DEG are under "AutoAim"; ALLIANCE and the start pose are under
 * "AutoAimTuningOpMode". Graph headingErrorDeg and turnPower while tuning. Start with
 * kP, add kD if it overshoots, and only add kS if it stalls a few degrees short.
 * Holding cross also drives to AIM_DISTANCE_IN from the goal: graph distanceErrorIn and
 * tune rangeP/rangeD the same way (set DRIVE_TO_DISTANCE false to tune turning alone).
 * Dashboard changes don't persist: copy the tuned values back into AutoAim.java.
 */
@Config
@TeleOp(name = "Auto Aim Tuning", group = "Tuning")
public class AutoAimTuningOpMode extends LinearOpMode {
    /** -1 for Blue, +1 for Red. Can be changed live. */
    public static double ALLIANCE = -1;
    /** true = aim the whole time, no button needed. */
    public static boolean AIM_WITHOUT_BUTTON = false;
    /** true = start from the pose auto (or the last TeleOp) saved, instead of START_X/Y/H. */
    public static boolean USE_SAVED_POSE = false;
    /** Start pose when USE_SAVED_POSE is false. Defaults to the auto start pose; START_Y is
     *  the Red value and is mirrored for Blue, like the auto path. */
    public static double START_X = -62, START_Y_RED = -12.5, START_H_DEG = 180;

    @Override
    public void runOpMode() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) {
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        }

        NeuronLocalizer localizer = new NeuronLocalizer(hardwareMap, telemetry, /*xOffsetMm=*/144.0, /*yOffsetMm=*/-144.0);
        NeuronDrive drive = new NeuronDrive(hardwareMap, localizer, /*trackWidthIn=*/405.0 / 25.4);
        AutoAim aim = new AutoAim(ALLIANCE);

        localizer.calibrate(true);
        Pose2D startPose = USE_SAVED_POSE
                ? (Pose2D) blackboard.getOrDefault(PlannerAuto.POSE_KEY, new Pose2D(0, 0, 0))
                : new Pose2D(START_X, START_Y_RED * (ALLIANCE >= 0 ? 1 : -1), Math.toRadians(START_H_DEG));
        localizer.setPosition(startPose);
        telemetry.addData("alliance", ALLIANCE >= 0 ? "RED" : "BLUE");
        telemetry.addData("start pose", "x %.1f, y %.1f, heading %.0f deg",
                startPose.x, startPose.y, Math.toDegrees(startPose.h));
        telemetry.update();

        waitForStart();

        boolean wasAiming = false;
        while (opModeIsActive()) {
            aim.setAlliance(ALLIANCE);
            drive.update();

            boolean aiming = AIM_WITHOUT_BUTTON || gamepad1.cross;
            double driveMax = NeuronDrive.teleOpMaxSpeed;
            double turn;
            if (aiming) {
                if (!wasAiming) aim.reset();
                // AutoAim is CCW-positive and gives the real motor power; drive() is
                // clockwise-positive and multiplies by driveMax, so flip and divide
                turn = -aim.update(drive.pose) / driveMax;
            } else {
                aim.updateTarget(drive.pose);
                turn = gamepad1.right_stick_x * Math.min(1, NeuronTeleOp.TURN_MAX_POWER / driveMax);
            }
            wasAiming = aiming;
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            if (aiming) {
                // Distance-to-goal drive from AutoAim (real powers, strafe left-positive)
                forward += aim.getForwardPower() / driveMax;
                strafe -= aim.getStrafeLeftPower() / driveMax;
            }
            drive.drive(forward, strafe, turn, driveMax);

            telemetry.addData("aiming", aiming);
            telemetry.addData("aimed", aim.isAimed());
            telemetry.addData("alliance", aim.getAlliance() > 0 ? "RED" : "BLUE");
            telemetry.addData("hive", aim.isPositiveSideHive() ? "positive x" : "negative x");
            telemetry.addData("goalX", aim.getGoalX());
            telemetry.addData("goalY", aim.getGoalY());
            telemetry.addData("x", drive.pose.x);
            telemetry.addData("y", drive.pose.y);
            telemetry.addData("headingDeg", Math.toDegrees(drive.pose.h));
            telemetry.addData("targetHeadingDeg", aim.getTargetHeadingDeg());
            telemetry.addData("headingErrorDeg", aim.getErrorDeg());
            telemetry.addData("turnPower", aim.getTurnPower());
            telemetry.addData("distanceIn", aim.getDistanceIn());
            telemetry.addData("distanceErrorIn", aim.getDistanceIn() - AutoAim.AIM_DISTANCE_IN);
            telemetry.addData("inRange", aim.isInRange());
            telemetry.addData("atDistance", aim.isAtDistance());
            telemetry.addData("inZone", aim.isInZone());
            telemetry.addData("spotX", aim.getSpotX());
            telemetry.addData("spotY", aim.getSpotY());
            telemetry.addData("forwardPower", aim.getForwardPower());
            telemetry.addData("strafeLeftPower", aim.getStrafeLeftPower());
            telemetry.update();
        }
    }
}
