package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.io.File;
import java.lang.reflect.Array;
import java.util.List;

public class odometryCalibrationLoop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);

    ElapsedTime timer = new ElapsedTime();

    double encoderCountsPerIn = 306.3816404153158;

    boolean doneTesting = false;

    double[] wheelBaseSeparationValues = {};
    double[] horizontalOffsetValues = {};

    File wheelBaseSeparationFile = AppUtil.getInstance().getSettingsFile("wheelBaseSeparation.txt");
    File horizontalTickOffsetFile = AppUtil.getInstance().getSettingsFile("horizontalTickOffset.txt");


    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);

        //// Makes the imu work upside down by assigning bytes to the register
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
        //Need to be in CONFIG mode to write to registers
        robot.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100); //Changing modes requires a delay before doing anything
        //Write to the AXIS_MAP_SIGN register
        robot.imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        //Need to change back into the IMU mode to use the gyro
        robot.imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100); //Changing modes again requires a delay

        // set telemetry
        telemetry.setMsTransmissionInterval(5);
        telemetry.addData("Status", "Waiting to be started");
        telemetry.update();

        waitForStart();

        while (!doneTesting) {
            // init the imu
            robot.initImu(hardwareMap, telemetry);
            sleep(500);

            // initialize some values
            double angle = robot.imu.getAngularOrientation().firstAngle;
            double firstAngle = angle;

            // do the turn
            while (angle < 90 && opModeIsActive()) {
                if (angle < 60) {
                    drive.circlepadMove(0, 0, robot.PIVOT_SPEED );
                } else {
                    drive.circlepadMove(0, 0, robot.PIVOT_SPEED * .85);
                }
            }
            drive.stop();

            // Record IMU and encoder values to calculate the constants
            angle = robot.imu.getAngularOrientation().firstAngle;
            timer.reset();
            while (timer.milliseconds() < 1000 && opModeIsActive()) {
                telemetry.addData("IMU Angle", angle);
                telemetry.update();
            }

            //Do the maths!
            double encoderDifference = Math.abs(robot.OLeft.getCurrentPosition()) +
                    Math.abs(robot.ORight.getCurrentPosition());
            double verticalEncoderTickOffsetPerDegree = encoderDifference / (robot.imu.getAngularOrientation().firstAngle - firstAngle); //changed from angle to imu.getangle
            double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * encoderCountsPerIn);
            double horizontalTickOffset = ((robot.OMiddle.getCurrentPosition()) / (Math.toRadians(robot.imu.getAngularOrientation().firstAngle - firstAngle)));

            telemetry.addData("Wheel base separation", wheelBaseSeparation);
            telemetry.addData("Horizontal tick offset per degree", horizontalTickOffset);
            telemetry.addLine("Press A to accept these values");
            telemetry.addLine("Press X to decline these values");
            telemetry.addLine("Press B and Y together to conclude testing");
            telemetry.update();
            while (true) {
                if (gamepad1.a) {
                    // TODO CODE TO ADD STUFF GOES HERE
                    break;
                }
                if (gamepad1.x) {
                    break;
                }
                if (gamepad1.b && gamepad1.y) {
                    doneTesting = true;
                    break;
                }
            }
        }
    }
}
