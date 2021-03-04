package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "Servo Test")
public class genericServoTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    Servo servo;

    double servoPosition = .5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        servo = robot.ShooterElevator;

        while (opModeIsActive()) {
            telemetry.addLine("Dpad left to go to 0, right for 1");
            telemetry.addLine("Dpad up to increase position, down to decrease\n");
            if (gamepad1.dpad_left) {
                servoPosition = 0;
                telemetry.addLine("Setting servo position to 0");
            } else if (gamepad1.dpad_right) {
                servoPosition = 1;
                telemetry.addLine("Setting servo position to 1");
            } else if (gamepad1.dpad_down) {
                servoPosition -= 0.004;
                telemetry.addLine("Decreasing servo position");
            } else if (gamepad1.dpad_up) {
                servoPosition += 0.004;
                telemetry.addLine("Increasing servo position");
            }
            telemetry.addData("servoPosition", servoPosition);
            telemetry.addData("The servo's getPosition", servo.getPosition());
            telemetry.update();

            servo.setPosition(servoPosition);
        }
    }
}
