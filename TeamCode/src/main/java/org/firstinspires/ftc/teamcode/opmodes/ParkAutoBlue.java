package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park Auto (Blue)", preselectTeleOp = "Neuron TeleOp (Blue)")
public class ParkAutoBlue extends ParkAuto {
    @Override
    protected double alliance() {
        return -1;
    }
}
