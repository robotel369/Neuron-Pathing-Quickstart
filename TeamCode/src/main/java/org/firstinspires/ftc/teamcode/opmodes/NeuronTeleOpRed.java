package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Neuron TeleOp for the Red alliance: auto aim (hold cross) always targets
 * the Red goal. Everything else is inherited from NeuronTeleOp.
 */
@TeleOp(name = "Neuron TeleOp (Red)")
public class NeuronTeleOpRed extends NeuronTeleOp {
    @Override
    protected Double fixedAlliance() {
        return 1.0;
    }
}
