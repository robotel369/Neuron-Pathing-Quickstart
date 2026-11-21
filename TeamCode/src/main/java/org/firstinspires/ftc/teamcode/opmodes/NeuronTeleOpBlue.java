package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Neuron TeleOp for the Blue alliance: auto aim (hold cross) always targets
 * the Blue goal. Everything else is inherited from NeuronTeleOp.
 */
@TeleOp(name = "Neuron TeleOp (Blue)")
public class NeuronTeleOpBlue extends NeuronTeleOp {
    @Override
    protected Double fixedAlliance() {
        return -1.0;
    }
}
