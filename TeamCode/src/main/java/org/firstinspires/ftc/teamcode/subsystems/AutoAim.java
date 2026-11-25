package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.neuronpathing.MathFunc;
import org.firstinspires.ftc.teamcode.neuronpathing.PID;
import org.firstinspires.ftc.teamcode.neuronpathing.Pose2D;

/**
 * Auto aim: turns the robot so the shooter points at the alliance goal, using the
 * Pinpoint pose, and drives it to a shooting spot that is AIM_DISTANCE_IN from the goal
 * (within +-AIM_DISTANCE_TOLERANCE_IN) AND inside the shooting zone in y
 * (Blue: +10 to +38, on its own goal's side; Red mirrored: -38 to -10). The spot keeps the closest y to the robot
 * (its own y if already in the zone, else the nearest zone y) and picks x on the 44 in arc. The OpMode adds the returned turn and forward/strafe
 * powers to the driver's sticks.
 *
 * Goal targets (inches, field coordinates):
 *   Blue: (-12, 14.5)   Red: (-12, -14.5), always the mirror of Blue
 * On the positive-x hive the goal y and zone flip sign (FLIP_Y_ON_POSITIVE_HIVE):
 *   Blue: (12, -14.5)   Red: (12, 14.5)
 * BIOBUZZ has two hive positions, so once the robot crosses into positive x the goal's
 * x switches to +12 (and back to -12 when it returns). A small hysteresis band around
 * HIVE_SWITCH_X stops the target from flickering when the robot sits right on the line.
 *
 * Default gains are copied from the values auto already drives with, so aiming turns
 * the same way auto does before any tuning:
 *   kP/kI/kD          = NeuronDrive hP/hI/hD (0.05 / 0 / 0.003, error in degrees)
 *   MAX_TURN_POWER    = PlannerAuto.MAX_SPEED (0.6); like auto, the PID output is
 *                       clipped to +-1 and then scaled by this
 *   AIMED_TOLERANCE   = NeuronDrive.closeHeadingToleranceDeg (2 deg)
 *   AIM_OFFSET_DEG    = 180: auto shoots facing heading 180 with the goal in the +x
 *                       direction, so the shooter fires out the back of the robot
 *   rangeP/I/D        = NeuronDrive forwardP/I/D (0.1 / 0 / 0.015, error in inches)
 *   MAX_RANGE_POWER   = PlannerAuto.MAX_SPEED (0.6)
 * Tune them live with AutoAimTuningOpMode + FTC Dashboard, then copy them back here.
 * Follows Neuron Pathing subsystem conventions.
 */
@Config
public class AutoAim {
    // ---- goal targets ------------------------------------------------------------
    /** Goal x while the robot is on the negative-x side of the field (first hive). */
    public static double GOAL_X_NEGATIVE_SIDE = -12;
    /** Goal x once the robot crosses into positive x (second hive). */
    public static double GOAL_X_POSITIVE_SIDE = 12;
    /** Blue goal y on the negative-x hive; Red always uses the mirror image (-BLUE_GOAL_Y),
     *  so tuning this sets both. */
    public static double BLUE_GOAL_Y = 14.5;
    /** true = the positive-x hive is rotated 180 deg from the negative-x one, so each
     *  alliance's goal y (and shooting zone) flips sign once the robot crosses into positive x.
     *  Without this, Blue aimed at Red's goal on the positive-x side. */
    public static boolean FLIP_Y_ON_POSITIVE_HIVE = true;
    /** The robot's x at which the goal switches hives. */
    public static double HIVE_SWITCH_X = 0;
    /** The robot has to be this many inches past HIVE_SWITCH_X before the goal switches. */
    public static double HIVE_SWITCH_HYSTERESIS_IN = 2;

    // ---- aiming ------------------------------------------------------------------
    /** Which side of the robot points at the goal: 0 = front, 180 = back (shooter side). */
    public static double AIM_OFFSET_DEG = 180;
    public static double kP = 0.05, kI = 0.0, kD = 0.003;
    /** Max turn power while aiming. */
    public static double MAX_TURN_POWER = 0.6;
    /** Minimum turn power while outside AIMED_TOLERANCE_DEG, to overcome friction on
     *  small errors. 0 = off (auto doesn't use one); try 0.05 if the robot stalls short. */
    public static double kS = 0.0;
    /** Counts as aimed when the heading is within this many degrees of the goal. */
    public static double AIMED_TOLERANCE_DEG = 2;

    // ---- distance to goal --------------------------------------------------------
    /** false = only turn, never drive to the shooting spot. */
    public static boolean DRIVE_TO_DISTANCE = true;
    /** Shooting distance from the robot's center to the goal target, in inches. */
    public static double AIM_DISTANCE_IN = 44;
    /** Counts as at distance within AIM_DISTANCE_IN +- this (3 in total window). */
    public static double AIM_DISTANCE_TOLERANCE_IN = 1.5;
    /** false = ignore the y zone and only hold the distance. */
    public static boolean USE_Y_ZONE = true;
    /** Shooting zone in y for Blue (same side as the Blue goal and the Blue auto path);
     *  Red uses the mirror image (-38 to -10). */
    public static double BLUE_ZONE_Y_MIN = 10, BLUE_ZONE_Y_MAX = 38;
    /** When the spot is clamped to a zone edge, aim this far inside the edge so the robot
     *  ends up clearly in the zone instead of creeping toward the line. */
    public static double ZONE_EDGE_MARGIN_IN = 1;
    public static double rangeP = 0.1, rangeI = 0.0, rangeD = 0.015;
    /** Max drive power toward/away from the goal. */
    public static double MAX_RANGE_POWER = 0.6;

    /** -1 for Blue, +1 for Red (same convention as PlannerAuto). */
    private double alliance;
    private boolean positiveSideHive = false;
    private PID pid = new PID(kP, kI, kD);
    private PID rangePid = new PID(rangeP, rangeI, rangeD);

    private double goalX, goalY;
    private double targetHeadingRad = 0;
    private double errorDeg = 0;
    private double turnPower = 0;
    private double distanceIn = 0;
    private double spotX = 0, spotY = 0;
    private double lastY = 0;
    private double forwardPower = 0;
    private double strafeLeftPower = 0;

    public AutoAim(double alliance) {
        setAlliance(alliance);
    }

    public void setAlliance(double alliance) {
        this.alliance = alliance >= 0 ? 1 : -1;
    }

    public double getAlliance() {
        return alliance;
    }

    /**
     * Call every loop (aiming or not) so the hive choice follows the robot around
     * the field. Updates the goal and the heading that would point at it.
     */
    public void updateTarget(Pose2D pose) {
        if (!positiveSideHive && pose.x > HIVE_SWITCH_X + HIVE_SWITCH_HYSTERESIS_IN) {
            positiveSideHive = true;
        } else if (positiveSideHive && pose.x < HIVE_SWITCH_X - HIVE_SWITCH_HYSTERESIS_IN) {
            positiveSideHive = false;
        }

        goalX = positiveSideHive ? GOAL_X_POSITIVE_SIDE : GOAL_X_NEGATIVE_SIDE;
        goalY = ySign() * BLUE_GOAL_Y;

        distanceIn = Math.hypot(goalX - pose.x, goalY - pose.y);
        lastY = pose.y;
        double headingToGoal = Math.atan2(goalY - pose.y, goalX - pose.x);
        targetHeadingRad = MathFunc.angleWrap(headingToGoal + Math.toRadians(AIM_OFFSET_DEG));
        errorDeg = Math.toDegrees(MathFunc.angleWrap(targetHeadingRad - pose.h));
    }

    /** Call once when aiming starts, so old PID state can't kick the robot. */
    public void reset() {
        pid = new PID(kP, kI, kD);
        rangePid = new PID(rangeP, rangeI, rangeD);
        turnPower = 0;
        forwardPower = 0;
        strafeLeftPower = 0;
    }

    /**
     * Updates the target and returns the turn power toward the goal, positive =
     * counter-clockwise (heading increasing), the same sign NeuronDrive's heading PID uses.
     */
    public double update(Pose2D pose) {
        updateTarget(pose);
        pid.updatePID(kP, kI, kD);

        double output = pid.update(errorDeg);
        if (Math.abs(errorDeg) > AIMED_TOLERANCE_DEG) {
            output += Math.signum(errorDeg) * kS;
        }
        turnPower = Range.clip(output, -1, 1) * MAX_TURN_POWER;

        updateRange(pose);
        return turnPower;
    }

    // Drives straight to the shooting spot (see findSpot()) until the robot is both at
    // distance and in the y zone. The power is worked out on the field and then turned into
    // robot-relative forward/strafe, so it works while the robot is turning.
    private void updateRange(Pose2D pose) {
        rangePid.updatePID(rangeP, rangeI, rangeD);
        findSpot(pose);
        double dx = spotX - pose.x, dy = spotY - pose.y;
        double spotDistance = Math.hypot(dx, dy);

        if (!DRIVE_TO_DISTANCE || isInRange() || spotDistance < 0.5) {
            rangePid.resetIntegral();
            forwardPower = 0;
            strafeLeftPower = 0;
            return;
        }

        double power = Range.clip(rangePid.update(spotDistance), 0, 1) * MAX_RANGE_POWER;
        double fieldX = dx / spotDistance * power;
        double fieldY = dy / spotDistance * power;
        // Field -> robot frame (same math NeuronDrive uses for forward/strafe error)
        double cosH = Math.cos(pose.h), sinH = Math.sin(pose.h);
        forwardPower = fieldX * cosH + fieldY * sinH;
        strafeLeftPower = -fieldX * sinH + fieldY * cosH;
    }

    // Shooting spot with the closest y: keep the robot's own y if it's already inside the
    // zone, otherwise use the nearest zone y (ZONE_EDGE_MARGIN_IN inside the edge). That y
    // is also kept within reach of the AIM_DISTANCE_IN circle. Then x is picked on the
    // circle, on the robot's side of the goal, so the robot mostly drives sideways in x
    // instead of sliding along the circle to some other y.
    private void findSpot(Pose2D pose) {
        double r = AIM_DISTANCE_IN;
        double targetY = pose.y;

        if (USE_Y_ZONE) {
            double zoneMin = zoneYMin(), zoneMax = zoneYMax();
            if (targetY < zoneMin) targetY = zoneMin + ZONE_EDGE_MARGIN_IN;
            if (targetY > zoneMax) targetY = zoneMax - ZONE_EDGE_MARGIN_IN;
        }

        // The circle only reaches goalY +- r; stay just inside that so x is well defined
        double reach = r - 0.5;
        if (targetY < goalY - reach) targetY = goalY - reach;
        if (targetY > goalY + reach) targetY = goalY + reach;

        double dy = targetY - goalY;
        double dx = Math.sqrt(r * r - dy * dy);
        double xA = goalX + dx, xB = goalX - dx;
        // Prefer the spot on the current hive's side of HIVE_SWITCH_X, so the goal doesn't
        // flip hives on the way there; otherwise take the one nearest the robot
        boolean aOk = positiveSideHive ? xA >= HIVE_SWITCH_X : xA <= HIVE_SWITCH_X;
        boolean bOk = positiveSideHive ? xB >= HIVE_SWITCH_X : xB <= HIVE_SWITCH_X;
        if (aOk != bOk) {
            spotX = aOk ? xA : xB;
        } else {
            spotX = Math.abs(xA - pose.x) <= Math.abs(xB - pose.x) ? xA : xB;
        }
        spotY = targetY;
    }

    /** Low edge of the y zone for this alliance (Red mirrors Blue across y = 0). */
    public double zoneYMin() {
        return ySign() > 0 ? BLUE_ZONE_Y_MIN : -BLUE_ZONE_Y_MAX;
    }

    /** High edge of the y zone for this alliance. */
    public double zoneYMax() {
        return ySign() > 0 ? BLUE_ZONE_Y_MAX : -BLUE_ZONE_Y_MIN;
    }

    // +1 = use the Blue numbers as written, -1 = mirrored in y. Red mirrors Blue, and on the
    // positive-x hive (if FLIP_Y_ON_POSITIVE_HIVE) both alliances flip again.
    private double ySign() {
        double sign = alliance > 0 ? -1 : 1;
        if (positiveSideHive && FLIP_Y_ON_POSITIVE_HIVE) sign = -sign;
        return sign;
    }

    /** True when the heading is on target and the robot is within distance tolerance. */
    public boolean isAimed() {
        return Math.abs(errorDeg) < AIMED_TOLERANCE_DEG && isInRange();
    }

    /** At shooting distance (+- tolerance) and, if USE_Y_ZONE, inside the y zone. */
    public boolean isInRange() {
        return !DRIVE_TO_DISTANCE || (isAtDistance() && isInZone());
    }

    public boolean isAtDistance() {
        return Math.abs(distanceIn - AIM_DISTANCE_IN) <= AIM_DISTANCE_TOLERANCE_IN;
    }

    public boolean isInZone() {
        return !USE_Y_ZONE || (lastY >= zoneYMin() && lastY <= zoneYMax());
    }

    public double getSpotX() {
        return spotX;
    }

    public double getSpotY() {
        return spotY;
    }

    /** Distance from the robot's center to the goal target, in inches. */
    public double getDistanceIn() {
        return distanceIn;
    }

    /** Real motor power forward (robot front), from update(). */
    public double getForwardPower() {
        return forwardPower;
    }

    /** Real motor power strafing left, from update(). */
    public double getStrafeLeftPower() {
        return strafeLeftPower;
    }

    public boolean isPositiveSideHive() {
        return positiveSideHive;
    }

    public double getGoalX() {
        return goalX;
    }

    public double getGoalY() {
        return goalY;
    }

    public double getTargetHeadingDeg() {
        return Math.toDegrees(targetHeadingRad);
    }

    public double getErrorDeg() {
        return errorDeg;
    }

    public double getTurnPower() {
        return turnPower;
    }
}
