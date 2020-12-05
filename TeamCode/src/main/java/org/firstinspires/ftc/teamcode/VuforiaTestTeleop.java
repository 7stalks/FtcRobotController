package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Vuforia Test")
public class VuforiaTestTeleop extends LinearOpMode {

    RobotHardware robot = new RobotHardware();
    GoBildaDrive drive = new GoBildaDrive(robot);
    VuforiaNavigation navigation = new VuforiaNavigation();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap, telemetry);
        robot.initVuforia(hardwareMap, telemetry);
        navigation.navigationInit(robot);

        waitForStart();

        while (opModeIsActive()) {
            navigation.navigation(telemetry);
            telemetry.update();
        }
    }
}
