package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

@TeleOp(name = "Odometry basic test")
public class OdometryBasicTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                odometryMove.diagonalToPoint(12, 0, 0);
            } else if (gamepad1.b) {
                odometryMove.diagonalToPoint(0, 12, 0);
            } else if (gamepad1.x) {
                odometryMove.diagonalToPoint(12, 12, 0);
            } else if (gamepad1.y) {
                odometryMove.diagonalToPoint(12, 12, Math.PI/2);
            }
            telemetry.addLine("Press A to move to 12 x, B to go to 12 y, X to go to " +
                    "(12, 12) with 0 theta, and Y to go to (12, 12) with theta = pi/2");
            odometry.queryOdometry();
        }
    }
}
