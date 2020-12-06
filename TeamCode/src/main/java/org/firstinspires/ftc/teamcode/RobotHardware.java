package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import static java.lang.Thread.*;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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
    public DcMotor WobbleMotor;

    // Odometers
    public DcMotor OLeft;
    public DcMotor ORight;
    public DcMotor OMiddle;

    // Servos
    public Servo ShooterServo;
    public Servo ShooterElevator;
    public Servo WobbleServo;
    public Servo WobbleCatcher;

    // Gyro (and temp sensor haha)
    public BNO055IMU bottom_imu;
    public BNO055IMU top_imu;

    public WebcamName frontWebcam;
    public WebcamName backWebcam;

    final public double stickThres = 0.05;
    final public double PIVOT_SPEED = -0.5;
    final public double SHOOTER_SERVO_START = 0.9;
    final public double SHOOTER_SERVO_MAX = 0.3;

    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    public static final float halfField = 72 * mmPerInch;
    public static final float quadField  = 36 * mmPerInch;

    private static final String VUFORIA_KEY = "AXl4o5z/////AAABmQyBF0iAaUTcguyLoBFeK1A7RHUVrQdTS" +
            "sPDqn4DelLm7BtbLuahVuZvBzuq5tPGrvi7D25P3xRzVgT1d+cADoNAMxuRVZs24o87S6gH0mM+Q/OrrQr5" +
            "7pTiumNffyuzBI728d+XgQJImM0rBxGcpwej8Ok0ZSCNIzzxVNf06dRwLEwu6jf0mCiA9yyffMFzreeL8UR" +
            "wm/xxuDsYxY7NrVtjlmslMTiu3nAUboaDP8jkhKvl8623x57MhYt4hof+iegRYjJzt+Knb5m5SfY5urWFGF" +
            "sLjZ4dqAhzXNiJmmKbKojUfjgvUld91gWm0UOXHkoezBuBVnLFasNmChD2uxpGGGeNdW1MvGitjFEvckKJ";

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;
    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public VuforiaLocalizer vuforia;
    public SwitchableCamera switchableCamera;

    public TFObjectDetector tensorFlowEngine;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";





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
            RightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        try {
            WobbleMotor = hardwareMap.get(DcMotor.class, "wobble_motor");
            WobbleMotor.setDirection(DcMotor.Direction.FORWARD);
            WobbleMotor.setPower(0);
            WobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            WobbleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: wobble motor identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: wobble motor not plugged in");    //
            WobbleMotor = null;
        }
        try {
            WobbleServo = hardwareMap.get(Servo.class, "wobble_servo");
            telemetry.addData("Status", "Servo: wobble servo identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: wobble serbo not plugged in");    //
            WobbleServo = null;
        }
        try {
            WobbleCatcher = hardwareMap.get(Servo.class, "wobble_catcher");
            telemetry.addData("Status", "Servo: wobble catcher identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: wobble catcher not plugged in");    //
            WobbleCatcher = null;
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

//        initVuforia(hardwareMap, telemetry);
//        initTFOD(telemetry);
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

    public void initVuforia(HardwareMap hardwareMap, Telemetry telemetry) {
        try {
            frontWebcam = hardwareMap.get(WebcamName.class, "front_webcam");
            backWebcam = hardwareMap.get(WebcamName.class, "back_webcam");
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = VUFORIA_KEY;
            parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(frontWebcam, backWebcam);
            vuforia = ClassFactory.getInstance().createVuforia(parameters);

            switchableCamera = (SwitchableCamera) vuforia.getCamera();
            switchableCamera.setActiveCamera(frontWebcam);

            telemetry.addData("Status", "Vuforia Initialized");
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Vuforia not enabled");
            telemetry.addData("err", err);
            vuforia = null;
        } catch (VuforiaException err) {
            telemetry.addData("Warning", "Vuforia Exception - not enabled");
            vuforia = null;
        }
    }

    public void initTFOD(Telemetry telemetry) {
        /* Initialize Tensor Flow Object Detection */
        if (vuforia != null) {
            int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
            tfodParameters.minResultConfidence = 0.7f;
            tensorFlowEngine = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
            tensorFlowEngine.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

            telemetry.addData("Status", "Tensor Flow Object Detection Initialized");
        } else {
            telemetry.addData("Status", "Tensor Flow Object Detection not Initialized");
        }
    }
}
