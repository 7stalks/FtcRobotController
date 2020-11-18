package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IntakeTest")
public class IntakeMotor extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive gobilda = new GoBildaDrive(robot);

    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            gobilda.circlepadMove(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            if (gamepad1.y) {
                robot.TopIntake.setPower(.85);
                robot.BottomIntake.setPower(1);
            } else  {
                robot.TopIntake.setPower(0);
                robot.BottomIntake.setPower(0);
            }

            if (gamepad1.left_bumper) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
            } else if (robot.ShooterServo.getPosition() >= robot.SHOOTER_SERVO_MAX) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
            }

            if (gamepad1.a) {
                robot.Shooter.setPower(1);
            } else {
                robot.Shooter.setPower(0);
            }
        }
    }
}
