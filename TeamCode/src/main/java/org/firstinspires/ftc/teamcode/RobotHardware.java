package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaException;

import java.nio.file.StandardWatchEventKinds;

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
    public DcMotor WobbleRotator;

    // Odometers
    public DcMotor OLeft;
    public DcMotor ORight;
    public DcMotor OMiddle;

    // Servos
    public Servo ShooterServo;
    public Servo ShooterElevator;
    public Servo WobbleServo;

    public Servo WobbleRotatorServo;
    public Servo WobbleCatcherFront;
    public Servo WobbleCatcherBack;

    // Gyro (and temp sensor haha)
    public BNO055IMU bottom_imu;
    public BNO055IMU top_imu;

    public WebcamName frontWebcam;
    public WebcamName backWebcam;

    final public double stickThres = 0.05;
    final public double PIVOT_SPEED = -0.5;
    final public double SHOOTER_SERVO_START = 0.88;
    final public double SHOOTER_SERVO_MAX = 0.32;

    final public double wobbleRotatorMin = 0;
    final public double wobbleRotatorMax = 1.0;
    final public double wobbleRotatorPickup = 0.24;
    final public double wobbleRotatorTop = 0.6;
    final public double wobbleCatcherFrontMin = 0.29;
    final public double wobbleCatcherFrontMax = 0.57;
    final public double wobbleCatcherBackMin = 0.27;
    final public double wobbleCatcherBackMax = 0.64;
    final public double wobbleCatcherFrontSpeed = (wobbleCatcherFrontMax-wobbleCatcherFrontMin)*0.0135135135;
    final public double wobbleCatcherBackSpeed = (wobbleCatcherBackMax-wobbleCatcherBackMin)*0.0135135135;

    ElapsedTime timer = new ElapsedTime();

    private static final String VUFORIA_KEY = "AXl4o5z/////AAABmQyBF0iAaUTcguyLoBFeK1A7RHUVrQdTS" +
            "sPDqn4DelLm7BtbLuahVuZvBzuq5tPGrvi7D25P3xRzVgT1d+cADoNAMxuRVZs24o87S6gH0mM+Q/OrrQr5" +
            "7pTiumNffyuzBI728d+XgQJImM0rBxGcpwej8Ok0ZSCNIzzxVNf06dRwLEwu6jf0mCiA9yyffMFzreeL8UR" +
            "wm/xxuDsYxY7NrVtjlmslMTiu3nAUboaDP8jkhKvl8623x57MhYt4hof+iegRYjJzt+Knb5m5SfY5urWFGF" +
            "sLjZ4dqAhzXNiJmmKbKojUfjgvUld91gWm0UOXHkoezBuBVnLFasNmChD2uxpGGGeNdW1MvGitjFEvckKJ";

    public BetterVuforia vuforia;
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

        // intake and shooter motors
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
            Shooter.setDirection(DcMotor.Direction.REVERSE);
            Shooter.setPower(0);
            Shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("Status", "Motor: shooter identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: shooter not plugged in");    //
            Shooter = null;
        }
        try {
            WobbleRotator = hardwareMap.get(DcMotor.class, "wobble_rotator");
            WobbleRotator.setDirection(DcMotor.Direction.FORWARD);
            WobbleRotator.setPower(0);
            WobbleRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            WobbleRotator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            WobbleRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            telemetry.addData("Status", "Motor: wobble rotator identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Motor: wobble rotator not plugged in");    //
            WobbleRotator = null;
        }

        // shooter and wobble servos
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
            WobbleRotatorServo = hardwareMap.get(Servo.class, "wobble_rotator");
            telemetry.addData("Status", "Servo: wobble rotator identified");    //
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: wobble rotator not plugged in");    //
            WobbleRotatorServo = null;
        }
        try {
            WobbleCatcherBack = hardwareMap.get(Servo.class, "wobble_catcher_back");
            telemetry.addData("Status", "Servo: wobble catcher back identified");    //
            WobbleCatcherBack.setPosition(wobbleCatcherBackMax);
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: wobble catcher back not plugged in");    //
            WobbleCatcherBack = null;
        }
        try {
            WobbleCatcherFront = hardwareMap.get(Servo.class, "wobble_catcher_front");
            telemetry.addData("Status", "Servo: wobble catcher front back identified");    //
            WobbleCatcherFront.setPosition(wobbleCatcherFrontMin);
        } catch (IllegalArgumentException err) {
            telemetry.addData("Warning", "Servo: wobble catcher front not plugged in");    //
            WobbleCatcherFront = null;
        }
        // naming for the odometers (they use the encoders on the controlhub)
        OLeft = RightFront;
        ORight = RightBack;
        OMiddle = LeftBack;

        // Init the IMUs/Gyros
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
        //end of robot init
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
            vuforia = new BetterVuforia(parameters);

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

    public void sleepTimer(int milliseconds, LinearOpMode opMode) {
        timer.reset();
        while (timer.milliseconds() <= milliseconds && opMode.opModeIsActive()) {opMode.idle();}
    }

    public void openWobble() {
        WobbleCatcherBack.setPosition(wobbleCatcherBackMin);
        WobbleCatcherFront.setPosition(wobbleCatcherFrontMax);
    }

    public void closWobble() {
        WobbleCatcherBack.setPosition(wobbleCatcherBackMax);
        WobbleCatcherFront.setPosition(wobbleCatcherFrontMin);
    }

    int lastPosition = 0;
    public void wobbleToPosition(int position, Telemetry telemetry) {
        int distanceToPosition;
        double power = 0;
        if (WobbleRotator.getCurrentPosition() < position - 5) {
            distanceToPosition = Math.abs(WobbleRotator.getCurrentPosition()) - Math.abs(position);
            power = .25 + (.75*distanceToPosition / 125);
            if (WobbleCatcherFront.getPosition() == wobbleCatcherFrontMin) {
                power = 1;
            }
            WobbleRotator.setPower(power);
        } else if (WobbleRotator.getCurrentPosition() > position + 5) {
            distanceToPosition = Math.abs(WobbleRotator.getCurrentPosition() - position);
            power = .25 + (.75 * distanceToPosition / 125);
            WobbleRotator.setPower(-power);
        } else {
            WobbleRotator.setPower(0);
        }
        telemetry.addData("power", power);
        lastPosition = WobbleRotator.getCurrentPosition();
    }
}
