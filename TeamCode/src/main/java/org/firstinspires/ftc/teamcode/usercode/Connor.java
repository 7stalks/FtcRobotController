package org.firstinspires.ftc.teamcode.usercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GoBildaDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;


public class Connor extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    Boolean shooterActive = false;


    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.a) {

               if (shooterActive) {
                   robot.Shooter.setPower(0);
                   shooterActive = false;
               } else {
                   robot.Shooter.setPower(1);
                   shooterActive = true;
               }

            }



        }
    }
}
