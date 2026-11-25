package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park Auto (Red)", preselectTeleOp = "Neuron TeleOp (Red)")
public class ParkAutoRed extends ParkAuto {
    @Override
    protected double alliance() {
        return 1;
    }
}
