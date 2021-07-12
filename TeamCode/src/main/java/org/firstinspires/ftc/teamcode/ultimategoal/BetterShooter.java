package org.firstinspires.ftc.teamcode.ultimategoal;

import android.util.Log;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class BetterShooter {

    RobotHardware robot;
    ShooterRpmThread shooterRpmThread;

    double power;
    double multiplier;

    public BetterShooter(RobotHardware robotHardware, ShooterRpmThread shooterRpmThread) {
        this.robot = robotHardware;
        this.shooterRpmThread = shooterRpmThread;
    }

    public void setRPM(double rpm) {
        if (Math.abs(shooterRpmThread.revolutionsPerMinute - rpm) > 800) {
            multiplier = 20;
        } else {
            multiplier = 1;
        }

        if (shooterRpmThread.revolutionsPerMinute - rpm > 150 && power > 0) {
            power -= 0.0006 * multiplier;
        } else if (shooterRpmThread.revolutionsPerMinute - rpm < -150 && power < 1) {
            power += 0.0006 * multiplier;
        }
        robot.Shooter.setPower(power);

        if (System.currentTimeMillis() % 3 == 0) {
            Log.v("SHOOTER", "power: " + power);
            Log.v("SHOOTER", "rpm: " + shooterRpmThread.revolutionsPerMinute);
            Log.v("SHOOTER", "--------");
        }
    }
}
