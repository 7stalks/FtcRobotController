package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.File;


@TeleOp(name = "Calibration de la Odometry")
public class OdometryCalibration extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);

    ElapsedTime timer = new ElapsedTime();

    double horizontalTickOffset = 0;

    // Text files to write the values to -- stored in robot controller Internal Storage\First\Settings
    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");

    double encoderCountsPerIn = 306.3816404153158;

    //5.42657564915

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

            double angle = robot.bottom_imu.getAngularOrientation().firstAngle;
            double firstAngle = angle;
            telemetry.addData("angle", angle);

            telemetry.setMsTransmissionInterval(5);
            telemetry.addData("Status", "Waiting to be started");
            telemetry.update();
            waitForStart();

            while (angle < 90 && opModeIsActive()) {
                if (angle < 60) {
                    drive.circlepadMove(0, 0, robot.PIVOT_SPEED );
                } else {
                    drive.circlepadMove(0, 0, robot.PIVOT_SPEED * .85);
                }

                angle = robot.bottom_imu.getAngularOrientation().firstAngle;

                telemetry.addData("IMU Angle", angle);
                telemetry.update();
            }
            drive.stop();
            timer.reset();
            // Record IMU and encoder values to calculate the constants
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
                telemetry.addData("IMU Angle", angle);
                telemetry.update();
            }

            //Encoder difference (leftEncoder - rightEncoder)
            double encoderDifference = Math.abs(robot.OLeft.getCurrentPosition()) +
                    Math.abs(robot.ORight.getCurrentPosition());
            double verticalEncoderTickOffsetPerDegree = encoderDifference / (robot.bottom_imu.getAngularOrientation().firstAngle - firstAngle); //changed from angle to imu.getangle
            double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * encoderCountsPerIn);

            horizontalTickOffset = ((robot.OMiddle.getCurrentPosition()) / (Math.toRadians(robot.bottom_imu.getAngularOrientation().firstAngle - firstAngle)));

        // Write constants to the text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        // Telemetry
        while (opModeIsActive()) {
//            telemetry.addData("Odometry Calibration Status", "Calibration Success");
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
//            telemetry.addData("base separation location", wheelBaseSeparationFile);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);
//            telemetry.addData("offset file locationm", horizontalTickOffsetFile);
//            telemetry.addData("IMU angle", angle);
            telemetry.addData("Current angle", robot.bottom_imu.getAngularOrientation().firstAngle);
            telemetry.addData("Left Position", robot.OLeft.getCurrentPosition());
            telemetry.addData("Right Position", robot.ORight.getCurrentPosition());
            telemetry.addData("Middle Position", robot.OMiddle.getCurrentPosition());
            telemetry.addData("initial angle", firstAngle);

            telemetry.update();
        }
    }
}
