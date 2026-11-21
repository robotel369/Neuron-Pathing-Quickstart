package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * NeuronTeleOp with different drive limits: always 100% drive and turning power.
 * Cross is auto aim (inherited), which replaces the old slow-turn-for-manual-aim.
 * Everything else (controls, shoot macro, heading light, pose from auto) is inherited from
 * NeuronTeleOp, so changes there carry over here automatically.
 */
@TeleOp(name = "wrooooom!!!")
public class WroomTeleOp extends NeuronTeleOp {
    @Override
    protected double driveMaxPower() {
        return 1.0;
    }

    @Override
    protected double turnMaxPower() {
        return 1.0;
    }
}
