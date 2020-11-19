package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.*;

public class RobotHardware {

    private HardwareMap hardwareMap = null;

    // Drive motors
    public DcMotor LeftFront;
    public DcMotor RightFront;
    public DcMotor LeftBack;
    public DcMotor RightBack;

    //Misc motors
    public DcMotor TopIntake;
    public DcMotor BottomIntake;
    public DcMotor Shooter;

    // Odometers
    public DcMotor OLeft;
    public DcMotor ORight;
    public DcMotor OMiddle;

    public Servo ShooterServo;
    public Servo ShooterElevator;

    // Gyro (and temp sensor haha)
    public BNO055IMU bottom_imu;
    public BNO055IMU top_imu;


    final public double stickThres = 0.05;
    final public double PIVOT_SPEED = -0.5;
    final public double SHOOTER_SERVO_START = 0.55;
    final public double SHOOTER_SERVO_MAX = 0.3;


    // This will be used on robotTeleop. Inits everything
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;

        // Mecanum motors initialization
        try {
            LeftFront = hardwareMap.get(DcMotor.class, "left_front");
            LeftFront.setDirection(DcMotor.Direction.FORWARD);
            LeftFront.setPower(0);
            LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            RightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            LeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            telemetry.addData("Status", "Motor: right_back identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: right_back not plugged in");    //
            RightBack = null;
        }
        try {
            TopIntake = hardwareMap.get(DcMotor.class, "top_intake");
            TopIntake.setDirection(DcMotor.Direction.FORWARD);
            TopIntake.setPower(0);
            TopIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TopIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: Top intake identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: Top intake not plugged in");    //
            TopIntake = null;
        }
        try {
            BottomIntake = hardwareMap.get(DcMotor.class, "bottom_intake");
            BottomIntake.setDirection(DcMotor.Direction.REVERSE);
            BottomIntake.setPower(0);
            BottomIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BottomIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: Bottom intake identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: Bottom intake not plugged in");    //
            BottomIntake = null;
        }
        try {
            Shooter = hardwareMap.get(DcMotor.class, "shooter");
            Shooter.setDirection(DcMotor.Direction.FORWARD);
            Shooter.setPower(0);
            Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: shooter identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: shooter not plugged in");    //
            Shooter = null;
        }
        try {
            ShooterServo = hardwareMap.get(Servo.class, "shooter_servo");
            telemetry.addData("Status", "Servo: Shooter servo identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: Shooter servo not plugged in");    //
            ShooterServo = null;
        }
        try {
            ShooterElevator = hardwareMap.get(Servo.class, "shooter_elevator");
            telemetry.addData("Status", "Servo: Shooter elevator identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: Shooter elevator not plugged in");    //
            ShooterElevator = null;
        }
        OLeft = RightFront;
        ORight = RightBack;
        OMiddle = LeftBack;

        // Init the IMU/Gyro
        try {
            bottom_imu = hardwareMap.get(BNO055IMU.class, "imu");

            //// Makes the imu work upside down by assigning bytes to the register
            byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis
            //Need to be in CONFIG mode to write to registers
            bottom_imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
            sleep(100); //Changing modes requires a delay before doing anything
            //Write to the AXIS_MAP_SIGN register
            bottom_imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
            //Need to change back into the IMU mode to use the gyro
            bottom_imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
            sleep(100); //Changing modes again requires a delay

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU_TOP";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            bottom_imu.initialize(parameters);
            telemetry.addData("Good", "Bottom Imu initialized");
        } catch (IllegalArgumentException | InterruptedException err) {
            telemetry.addData("Warning", "Bottom Imu not initialized");
        }

        try {
            top_imu = hardwareMap.get(BNO055IMU.class, "imu 1");

            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "IMU_TOP";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            top_imu.initialize(parameters);
            telemetry.addData("Good", "Top Imu initialized");
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Top Imu not initialized");
        }
    }

    // Inits just the mecanum drive (nothing else)
    public void initMecanum(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
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
