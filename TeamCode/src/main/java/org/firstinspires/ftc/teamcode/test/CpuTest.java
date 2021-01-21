package org.firstinspires.ftc.teamcode.test;


import android.util.Log;
import android.util.LogPrinter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

//@TeleOp(name = "Cpu test")
public class CpuTest {

//    Runtime runtime = Runtime.getRuntime();
//    String TAG = "MEMORY STUFF ";
//    Process output;

    public static void run_cmd(String[] commands) throws InterruptedException, IOException {

        Log.i("Run CMD", commands[0]);
        final Runtime r = Runtime.getRuntime();
        final Process p = r.exec(commands);
        final int returnCode = p.waitFor();

        String line = "";
        final BufferedReader is = new BufferedReader(new InputStreamReader(p.getInputStream()));
        while (true) {
            line = is.readLine();
            if (line == null) {
                break;
            }
            Log.i("Run CMD STDOUT", line);
        }

        final BufferedReader is2 = new BufferedReader(new InputStreamReader(p.getErrorStream()));
        while (true) {
            line = is2.readLine();
            if (line == null) {
                break;
            }
            Log.i("Run CMD STDERR", line);
        }
    }

    public void doDebug() {
        String[] commands = {"free", "-h"};
        try {
            run_cmd(commands);
        } catch (IOException | InterruptedException e) {
            e.printStackTrace();
        }
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        waitForStart();
//        int counter = 0;
//        while (opModeIsActive()) {
//            String[] commands = {"free", "-h"};
//            try {
//                run_cmd(commands);
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//            break;
//        }
//    }
}