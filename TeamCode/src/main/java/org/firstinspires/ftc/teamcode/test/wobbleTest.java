package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

@TeleOp(name = "wobble test")
public class wobbleTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    ElapsedTime myTimer = new ElapsedTime();

    boolean wobbleOn = false;
    double wobblePower;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(5);
        telemetry.update();
        myTimer.reset();

        waitForStart();

        while (opModeIsActive()) {

            if (myTimer.milliseconds() > 250) {
                if (gamepad1.a) {
                    wobbleOn = !wobbleOn;
                    wobblePower = .15;
                    myTimer.reset();
                } else if (gamepad1.b) {
                    wobbleOn = !wobbleOn;
                    wobblePower = -.15;
                    myTimer.reset();
                }
            }

            if (wobbleOn) {
                if ((!robot.topWobbleLimit.getState() && wobblePower > 0) || (!robot.bottomWobbleLimit.getState() && wobblePower < 0)) {
                    wobblePower = 0;
                }
                robot.WobbleRotator.setPower(wobblePower);
            } else {
                robot.WobbleRotator.setPower(0);
            }

            telemetry.addData("is the top limit pressed?", !robot.topWobbleLimit.getState());
            telemetry.addData("is the bottom limit pressed?", !robot.bottomWobbleLimit.getState());
            telemetry.addData("wobble position", robot.WobbleRotator.getCurrentPosition());
            telemetry.update();
        }
    }
}
