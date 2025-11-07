package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="LIFT SYNC TEST", group="Tests")
public class LiftSystem extends OpMode {

    private RobotHardware hardware;
    private boolean lastBumper = false;
    private static final int LIFT_TARGET = 3600;   // change as needed

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        // Sync both winches at start
        hardware.lWinch.setTargetPosition(LIFT_TARGET);
        hardware.rWinch.setTargetPosition(LIFT_TARGET);
        hardware.lWinch.setPower(0);
        hardware.rWinch.setPower(0);

        telemetry.addData("LIFT SYNC", "LEFT BUMPER = raise");
        telemetry.addData("Target", LIFT_TARGET);
    }

    @Override
    public void loop() {
        boolean bumper = gamepad2.left_bumper;

        // ---- PRESS BUMPER → RAISE ----
        if (bumper && !lastBumper) {
            hardware.lWinch.setTargetPosition(LIFT_TARGET);
            hardware.rWinch.setTargetPosition(LIFT_TARGET);
            hardware.lWinch.setPower(0.4);
            hardware.rWinch.setPower(0.4);
            telemetry.addData("LIFT", "GOING UP");
        }
        lastBumper = bumper;

        // ---- SYNC IN REAL TIME ----
        int lPos = hardware.lWinch.getCurrentPosition();
        int rPos = hardware.rWinch.getCurrentPosition();
        int diff = Math.abs(lPos - rPos);

        if (diff > 50) {
            if (lPos > rPos) {
                hardware.rWinch.setPower(0.45);  // catch up
            } else {
                hardware.lWinch.setPower(0.45);
            }
        } else {
            hardware.lWinch.setPower(0.4);
            hardware.rWinch.setPower(0.4);
        }

        // ---- STOP WHEN DONE ----
        if (!hardware.lWinch.isBusy() && !hardware.rWinch.isBusy()) {
            hardware.lWinch.setPower(0);
            hardware.rWinch.setPower(0);
        }

        // ---- TELEMETRY ----
        telemetry.addData("L Pos", lPos);
        telemetry.addData("R Pos", rPos);
        telemetry.addData("Diff", diff);
        telemetry.addData("L Busy", hardware.lWinch.isBusy() ? "YES" : "DONE");
        telemetry.addData("R Busy", hardware.rWinch.isBusy() ? "YES" : "DONE");
        telemetry.addData("Power L", "%.2f", hardware.lWinch.getPower());
        telemetry.addData("Power R", "%.2f", hardware.rWinch.getPower());
    }
}