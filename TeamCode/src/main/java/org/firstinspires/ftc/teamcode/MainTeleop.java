package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main")
public class MainTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);

    boolean wobbleDown = true;
    boolean wobbleCaught = false;
    private float i = 0;

    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {

            drive.circlepadMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);

            drive.dpadMove(gamepad1.dpad_up, gamepad1.dpad_right, gamepad1.dpad_left, gamepad1.dpad_down);

            if (gamepad2.a && i == 0) {
                robot.TopIntake.setPower(.85);
                robot.BottomIntake.setPower(1);
                i = 1;
            } else if (gamepad2.a && i == 1) {
                robot.TopIntake.setPower(0);
                robot.BottomIntake.setPower(0);
                i = 0;
                }
            }

            if (gamepad2.left_bumper) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
            } else if (robot.ShooterServo.getPosition() >= robot.SHOOTER_SERVO_MAX) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
            }

            if (gamepad2.right_trigger > .1) {
                robot.Shooter.setPower(1);
            } else {
                robot.Shooter.setPower(0);
            }

            if (gamepad2.dpad_up) {
                robot.WobbleMotor.setPower(.8);
            } else {
                robot.WobbleMotor.setPower(0);
            }

            if (gamepad2.dpad_down) {
                robot.WobbleMotor.setPower(-.8);
            } else {
                robot.WobbleMotor.setPower(0);
            }

            if (gamepad2.dpad_left) {
                if (wobbleDown) {
                    robot.WobbleServo.setPosition(1);
                    wobbleDown = false;
                } else {
                    robot.WobbleServo.setPosition(0);
                    wobbleDown = true;
                }
            }

            if (gamepad2.dpad_right) {
                if (!wobbleCaught) {
                    robot.WobbleCatcher.setPosition(1);
                    wobbleCaught = true;
                    sleep(20);
                } else {
                    robot.WobbleCatcher.setPosition(0);
                    wobbleCaught = false;
                }
            }
        }
    }