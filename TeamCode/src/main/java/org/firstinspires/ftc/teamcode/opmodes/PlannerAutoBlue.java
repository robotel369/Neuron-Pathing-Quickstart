package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Planner Auto (Blue)", preselectTeleOp = "Neuron TeleOp (Basic)")
public class PlannerAutoBlue extends PlannerAuto {
    @Override
    protected double alliance() {
        return -1;
    }
}
