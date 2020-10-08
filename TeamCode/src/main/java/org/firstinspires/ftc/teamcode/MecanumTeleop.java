package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Teleop", group = "Robot")
public class MecanumTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive gobilda = new GoBildaDrive(robot);

    @Override
    // Insert anything to be done as soon as INIT is pressed
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(1);
        int i = 1;


        // Insert anything to be done as soon as START is hit before the while loop
        waitForStart();

        // Insert anything to be iterated in a loop
        while (opModeIsActive()) {
            // Does the circle pad movements and dpad drive (one or the other)
            // Gives priority to dpad move because it's after and the power values will trump the
            // circle pad ones
            telemetry.addData("counter", i);
            telemetry.update();
            i++;
            gobilda.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x,
                    gamepad1.right_stick_x);
            gobilda.dpadMove(gamepad1.dpad_right, gamepad1.dpad_up, gamepad1.dpad_left,
                    gamepad1.dpad_down);
        }
    }
}
