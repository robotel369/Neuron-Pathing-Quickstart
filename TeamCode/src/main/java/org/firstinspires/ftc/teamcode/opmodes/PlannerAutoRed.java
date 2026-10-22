package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Planner Auto (Red)", preselectTeleOp = "Neuron TeleOp (Basic)")
public class PlannerAutoRed extends PlannerAuto {
    @Override
    protected double alliance() {
        return 1;
    }
}
