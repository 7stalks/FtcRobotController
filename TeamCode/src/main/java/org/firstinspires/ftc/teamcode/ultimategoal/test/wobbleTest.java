package org.firstinspires.ftc.teamcode.ultimategoal.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;

@TeleOp(name = "wobble test")
public class wobbleTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();

    double wobblePower;
    int position = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.initWobble();
        telemetry.setMsTransmissionInterval(5);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up && position < 0) {
                position += 35;
            } else if (gamepad1.dpad_down && position > robot.wobbleRotatorMinimum) {
                position -= 35;
            }

            if (gamepad1.a) {
                position = robot.wobbleRotatorMinimum;
            } else if (gamepad1.b) {
                position = robot.wobbleRotatorFullUp;
            } else if (gamepad1.x) {
                position = robot.wobbleRotatorUp;
            }
            telemetry.addData("position", position);
            telemetry.addData("wobble position", robot.getWobblePosition());
            robot.wobbleSetPosition(position);


            if (gamepad2.a) {
                if (!(robot.WobbleCatcherBack.getPosition() > robot.wobbleCatcherBackMax)) {
                    robot.WobbleCatcherBack.setPosition(robot.WobbleCatcherBack.getPosition() + robot.wobbleCatcherBackSpeed);
                    robot.WobbleCatcherFront.setPosition(robot.WobbleCatcherFront.getPosition() + robot.wobbleCatcherFrontSpeed);
                }
            } else if (gamepad2.b) {
                if (!(robot.WobbleCatcherBack.getPosition() < robot.wobbleCatcherBackMin)) {
                    robot.WobbleCatcherBack.setPosition(robot.WobbleCatcherBack.getPosition() - robot.wobbleCatcherBackSpeed);
                    robot.WobbleCatcherFront.setPosition(robot.WobbleCatcherFront.getPosition() - robot.wobbleCatcherFrontSpeed);
                }
            }
            telemetry.addData("top limit", robot.topWobbleLimit.getState());
            telemetry.addData("bottom limit", robot.bottomWobbleLimit.getState());
            telemetry.addData("wobble power", wobblePower);
            telemetry.addData("Wobble catcher back", robot.WobbleCatcherBack.getPosition());
            telemetry.addData("Wobble catcher front", robot.WobbleCatcherFront.getPosition());
            telemetry.update();
        }
    }
}
