package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vuforia Test")
public class VuforiaTestTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    VuforiaNavigation navigation = new VuforiaNavigation();

    boolean vuforia = true;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.initVuforia(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                vuforia = true;
            } else if (gamepad1.b) {
                vuforia = false;
            }

            if (vuforia) {
                if (robot.switchableCamera.getActiveCamera() != robot.backWebcam) {
                    if (robot.tensorFlowEngine != null) {
                        robot.tensorFlowEngine.deactivate();
                    }
                    robot.switchableCamera.setActiveCamera(robot.backWebcam);
                    navigation.navigationInit(robot);
                }
                navigation.navigation(telemetry);
            } else {
                if (robot.switchableCamera.getActiveCamera() != robot.frontWebcam) {
                    robot.switchableCamera.setActiveCamera(robot.frontWebcam);
                    robot.initTFOD(telemetry);
                    robot.tensorFlowEngine.activate();
                }
            }
            telemetry.update();
        }
    }
}
