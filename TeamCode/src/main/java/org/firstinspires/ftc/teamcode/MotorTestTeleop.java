package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Motor Test", group = "Robot")
public class MotorTestTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive gobilda = new GoBildaDrive(robot);

    @Override
    // Insert anything to be done as soon as INIT is pressed
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        // Insert anything to be done as soon as START is hit before the while loop
        waitForStart();

        // Insert anything to be iterated in a loop
        while (opModeIsActive()) {
            // Does the circle pad movements and dpad drive (one or the other)
            // Gives priority to dpad move because it's after and the power values will trump the
            // circle pad ones
            telemetry.addData("Hey", "I'm working");
            telemetry.addData("leftstick x", gamepad1.left_stick_x);
            telemetry.addData("leftstick y", gamepad1.left_stick_y);
            telemetry.addData("rightstick x", gamepad1.right_stick_x);
            telemetry.update();
            gobilda.motorTest(gamepad1.y, gamepad1.b, gamepad1.a, gamepad1.x);
        }
    }
}
