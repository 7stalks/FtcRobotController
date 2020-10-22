package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class IntakeMotor extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive gobilda = new GoBildaDrive(robot);

    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            gobilda.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.y = true) {
                robot.Intake.setPower(1);

            } else  {
                robot.Intake.setPower(0);
            }



        }


    }


}
