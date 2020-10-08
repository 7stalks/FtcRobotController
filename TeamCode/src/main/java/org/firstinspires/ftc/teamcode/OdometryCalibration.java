package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

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

    double encoderCountsPerIn = 307.699557;

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


        double angle = robot.imu.getAngularOrientation().firstAngle;
        telemetry.addData("angle", angle);

        telemetry.setMsTransmissionInterval(10);
        telemetry.addData("Status", "Waiting to be started");
        telemetry.update();
        waitForStart();

        while (angle < 90 && opModeIsActive()) {
            if (angle < 60) {
                drive.circlepadMove(0, 0, robot.PIVOT_SPEED);
            } else {
                drive.circlepadMove(0, 0, robot.PIVOT_SPEED / 2);
            }

            angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC.reverse(), AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

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
        double verticalEncoderTickOffsetPerDegree = encoderDifference / angle;
        double wheelBaseSeparation = (2 * 90 * verticalEncoderTickOffsetPerDegree) / (Math.PI * encoderCountsPerIn);

        horizontalTickOffset = ((robot.OMiddle.getCurrentPosition()) / (Math.toRadians(robot.imu.getAngularOrientation().firstAngle)));

        // Write constants to the text files
        ReadWriteFile.writeFile(wheelBaseSeparationFile, String.valueOf(wheelBaseSeparation));
        ReadWriteFile.writeFile(horizontalTickOffsetFile, String.valueOf(horizontalTickOffset));

        // Telemetry
        while (opModeIsActive()) {
            telemetry.addData("Odometry Calibration Status", "Calibration Success");
            telemetry.addData("Wheel Base Separation", wheelBaseSeparation);
            telemetry.addData("base separation location", wheelBaseSeparationFile);
            telemetry.addData("Horizontal Encoder Offset", horizontalTickOffset);
            telemetry.addData("offset file locationm", horizontalTickOffsetFile);
            telemetry.addData("IMU angle", angle);
            telemetry.addData("NEW IMY ANGLE LOOK HERE", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Left Position", robot.OLeft.getCurrentPosition());
            telemetry.addData("Right Position", robot.ORight.getCurrentPosition());
            telemetry.addData("Middle Position", robot.OMiddle.getCurrentPosition());

            telemetry.update();
        }
    }
}
