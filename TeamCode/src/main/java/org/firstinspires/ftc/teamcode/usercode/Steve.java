package org.firstinspires.ftc.teamcode.usercode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Steven Test", group = "Steve")
public class Steve extends LinearOpMode {
    DcMotor motor;
    Servo servo;

    double counter = 0.0;
    double SHOOTER_START = 0.50;
    double SHOOTER_MAX = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        motor = hardwareMap.get(DcMotor.class, "motor");

        servo.setPosition(SHOOTER_START);

        telemetry.addData(">", "Press Start to scan servo and motor...");
        telemetry.update();
        waitForStart();

        counter = SHOOTER_START;
        while (opModeIsActive()) {
            
            if (gamepad1.left_bumper) {
                servo.setPosition(SHOOTER_MAX);
            } else if (servo.getPosition() >= SHOOTER_MAX) {
                servo.setPosition(SHOOTER_START);
            }

            telemetry.addData("Servo Limit", servo.getPosition());
            telemetry.update();
        }
    }
}
