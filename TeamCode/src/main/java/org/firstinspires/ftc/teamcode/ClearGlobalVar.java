package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ClearGlobalVar extends OpMode {

    @Override
    public void init() {

        Global.pose = null;
    }

    @Override
    public void loop() {

    }
}
