package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.ShooterRpmThread;
import org.firstinspires.ftc.teamcode.BetterShooter;

@TeleOp(name = "an rpm shoot test")
public class RpmShootTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    ShooterRpmThread shooterThread = new ShooterRpmThread(robot, this);
    BetterShooter betterShooter = new BetterShooter(robot, shooterThread);

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        shooterThread.start();
        telemetry.setMsTransmissionInterval(5);
        robot.ShooterElevator.setPosition(.53);
        waitForStart();

        while (opModeIsActive()) {
            betterShooter.setRPM(5200 * gamepad2.left_trigger);
            if (gamepad2.right_trigger > .1 && shooterThread.revolutionsPerMinute > 4800) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_MAX);
            } else if (robot.ShooterServo.getPosition() <= robot.SHOOTER_SERVO_START) {
                robot.ShooterServo.setPosition(robot.SHOOTER_SERVO_START);
            }
            telemetry.addData("gamepad 2 left trigger", gamepad2.left_trigger);
            telemetry.addData("5200 times that", 5200 * gamepad2.left_trigger);
            telemetry.addData("RPM", shooterThread.revolutionsPerMinute);
            telemetry.update();
        }
    }
}
