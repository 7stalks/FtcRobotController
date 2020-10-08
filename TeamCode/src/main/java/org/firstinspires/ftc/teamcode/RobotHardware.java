package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {

    HardwareMap hardwareMap = null;

    // Mecanum motors
    public DcMotor LeftFront;
    public DcMotor RightFront;
    public DcMotor LeftBack;
    public DcMotor RightBack;

    // Odometers
    public DcMotor OLeft;
    public DcMotor ORight;
    public DcMotor OMiddle;

    // Gyro (and temp sensor haha)
    BNO055IMU imu;


    final public double stickThres = .2;

    // This will be used on robotTeleop. Inits everything
    public void init(HardwareMap hardware_map, Telemetry telemetry) {
        hardwareMap = hardware_map;

        // Mecanum motors initialization
        try {
            LeftFront = hardwareMap.get(DcMotor.class, "left_front");
            LeftFront.setDirection(DcMotor.Direction.FORWARD);
            LeftFront.setPower(0);
            LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: left_front identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: left_front not plugged in");    //
            LeftFront = null;
        }
        try {
            RightFront = hardwareMap.get(DcMotor.class, "right_front");
            RightFront.setDirection(DcMotor.Direction.REVERSE);
            RightFront.setPower(0);
            RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: right_front identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: right_front not plugged in");    //
            RightFront = null;
        }
        try {
            LeftBack = hardwareMap.get(DcMotor.class, "left_back");
            LeftBack.setDirection(DcMotor.Direction.FORWARD);
            LeftBack.setPower(0);
            LeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: left_back identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: left_back not plugged in");    //
            LeftBack = null;
        }
        try {
            RightBack = hardwareMap.get(DcMotor.class, "right_back");
            RightBack.setDirection(DcMotor.Direction.REVERSE);
            RightBack.setPower(0);
            RightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: right_back identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: right_back not plugged in");    //
            RightBack = null;
        }

        ORight = RightBack;
        OLeft = LeftFront;
        OMiddle = LeftBack;

//        // Odometry initialization
//        try {
//            OLeft = hardwareMap.get(DcMotor.class, "left_front");
//            OLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            OLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            OLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            telemetry.addData("Good", "o_left identified");    //
//        } catch (IllegalArgumentException err) {
//            telemetry.addData("Warning", "Odometry: o_left not plugged in");    //
//            OLeft = null;
//        }
//        try {
//            ORight = hardwareMap.get(DcMotor.class, "right_back");
//            ORight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            ORight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            ORight.setDirection(DcMotorSimple.Direction.FORWARD);
//            telemetry.addData("Good", "o_right identified");    //
//        } catch (IllegalArgumentException err) {
//            telemetry.addData("Warning", "Odometry: o_right not plugged in");    //
//            ORight = null;
//        }
//        try {
//            OMiddle = hardwareMap.get(DcMotor.class, "left_back");
//            OMiddle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            OMiddle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            OMiddle.setDirection(DcMotorSimple.Direction.REVERSE);
//            telemetry.addData("Good", "o_middle identified");    //
//        } catch (IllegalArgumentException err) {
//            telemetry.addData("Warning", "Odometry: o_middle not plugged in");    //
//            OMiddle = null;
//        }

        // Init the IMU/Gyro
        try {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            imu = hardware_map.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            telemetry.addData("Good", "Imu initialized");
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Imu not initialized");
        }
    }


    // Inits just the mecanum drive (nothing else)
    public void initMecanum(HardwareMap hardware_map, Telemetry telemetry) {
        hardwareMap = hardware_map;
        try {
            LeftFront = hardwareMap.get(DcMotor.class, "left_front");
            LeftFront.setDirection(DcMotor.Direction.FORWARD);
            LeftFront.setPower(0);
            LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: left_front identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: left_front not plugged in");    //
            LeftFront = null;
        }
        try {
            RightFront = hardwareMap.get(DcMotor.class, "right_front");
            RightFront.setDirection(DcMotor.Direction.REVERSE);
            RightFront.setPower(0);
            RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: right_front identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: right_front not plugged in");    //
            RightFront = null;
        }
        try {
            LeftBack = hardwareMap.get(DcMotor.class, "left_back");
            LeftBack.setDirection(DcMotor.Direction.FORWARD);
            LeftBack.setPower(0);
            LeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: left_back identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: left_back not plugged in");    //
            LeftBack = null;
        }
        try {
            RightBack = hardwareMap.get(DcMotor.class, "right_back");
            RightBack.setDirection(DcMotor.Direction.REVERSE);
            RightBack.setPower(0);
            RightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: right_back identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: right_back not plugged in");    //
            RightBack = null;
        }
    }
}
