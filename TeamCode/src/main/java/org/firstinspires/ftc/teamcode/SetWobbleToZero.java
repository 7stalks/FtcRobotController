package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Init Wobble")
public class SetWobbleToZero extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        waitForStart();
        robot.initWobble();
    }
}
