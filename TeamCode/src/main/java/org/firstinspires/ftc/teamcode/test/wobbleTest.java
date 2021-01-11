package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.odometry.OdometryMove;

@TeleOp(name = "wobble test")
public class wobbleTest extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    Odometry odometry = new Odometry(robot, telemetry);
    OdometryMove odometryMove = new OdometryMove(this, robot, odometry);

    boolean goToPosition = false;
    boolean goToHighPosition = false;
    boolean closeWobble = false;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.openWobble();
        telemetry.setMsTransmissionInterval(5);
        telemetry.update();

        odometryMove.rotate(Math.PI/2);
        waitForStart();

        while (opModeIsActive()) {
//            if (gamepad1.a) {
//                goToPosition = !goToPosition;
//                goToHighPosition = false;
//            }
//            if (goToPosition) {
//                robot.wobbleToPosition(-150, telemetry);
//            }
//            if (gamepad1.x) {
//                goToHighPosition = !goToHighPosition;
//                goToPosition = false;
//            }
//            if (goToHighPosition) {
//                robot.wobbleToPosition(-80, telemetry);
//            }
//            if (gamepad1.b) {
//                closeWobble = !closeWobble;
//            }
//            if (closeWobble) {
//                robot.closWobble();
//            } else {
//                robot.openWobble();
//            }
//            telemetry.addData("wobble position", robot.WobbleRotator.getCurrentPosition());
//            telemetry.update();
////            robot.WobbleRotator.setPower(1);
//        }


            if (gamepad1.a) {
                odometryMove.testDoubleStrafeToPoint(12, 12, Math.PI / 2);
            }
            odometry.queryOdometry();
        }
    }
}
