package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Calibration de la Odometry")
public class OdometryCalibration extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);

    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        double slowpower = .3;

        telemetry.setMsTransmissionInterval(2);
        telemetry.addData("Status", "Waiting to be started");
        telemetry.update();

        double angle = robot.imu.getAngularOrientation().firstAngle;

        waitForStart();

        while (opModeIsActive()) {
            double initAng = robot.imu.getAngularOrientation().firstAngle;
            while (angle < (initAng + 90)) {

                robot.RightFront.setPower(slowpower);
                robot.LeftFront.setPower(-slowpower);
                robot.RightBack.setPower(slowpower);
                robot.LeftFront.setPower(-slowpower);

                // left, right, mid, angle
                double[] odometryInfo = {robot.OLeft.getCurrentPosition(), robot.ORight.getCurrentPosition(), robot.OMiddle.getCurrentPosition()};

                telemetry.addData("Right Odometer", odometryInfo[0]);
                telemetry.addData("Left Odometer", odometryInfo[1]);
                telemetry.addData("Middle Odometer", odometryInfo[2]);
                telemetry.addData("Angle", angle);
                telemetry.update();

                angle = robot.imu.getAngularOrientation().firstAngle;
            }
            drive.stop();

        }
    }

}
