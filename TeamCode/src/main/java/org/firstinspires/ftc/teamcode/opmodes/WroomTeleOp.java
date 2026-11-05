package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * NeuronTeleOp with different drive limits: always 100% drive power, and turning at 100%
 * except while cross is held, when it drops to SLOW_TURN_MAX_POWER.
 * Everything else (controls, shoot macro, arm, shooter pivot, pose from auto) is inherited from
 * NeuronTeleOp, so changes there carry over here automatically.
 */
@TeleOp(name = "wrooooom!!!")
public class WroomTeleOp extends NeuronTeleOp {
    /** Max turning power while cross is held. */
    public static double SLOW_TURN_MAX_POWER = 0.3;

    @Override
    protected double driveMaxPower(boolean crossHeld) {
        return 1.0;
    }

    @Override
    protected double turnMaxPower(boolean crossHeld) {
        return crossHeld ? SLOW_TURN_MAX_POWER : 1.0;
    }
}
