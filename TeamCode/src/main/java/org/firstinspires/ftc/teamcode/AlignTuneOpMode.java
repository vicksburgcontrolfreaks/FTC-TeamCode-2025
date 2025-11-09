package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Tune: AprilTag Align", group = "Tuning")
public class AlignTuneOpMode extends OpMode {

    private RobotHardware hardware;
    private ShootAprilTag aligner;
    private Follower follower;

    // === TUNING PARAMETERS (Adjust in init_loop) ===
    private double kp = 1.0;           // Proportional gain
    private double maxSpeed = 1.0;     // Max rotation speed
    private double tolerance = 23.0;   // Pixel tolerance
    private double timeoutSec = 3.0;   // Alignment timeout
    private int tagId = 20;            // Blue: 20, Red: 24

    // === CONTROL ===
    private boolean alignActive = false;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        try {
            follower = Constants.createFollower(hardwareMap);
            telemetry.addData("Status", "Follower OK");
        } catch (Exception e) {
            telemetry.addData("Error", "Follower failed: " + e.getMessage());
        }

        aligner = new ShootAprilTag(hardware, follower, telemetry);
        aligner.setTelemetryEnabled(true);

        telemetry.addData("TUNE MODE", "Use Gamepad2 to adjust");
        telemetry.addData("Start Align", "Gamepad1: A");
        telemetry.addData("Stop Align", "Gamepad1: B");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // === ADJUST TUNING VALUES ===
        if (gamepad2.dpad_up) {
            kp = Math.max(0.1, kp + 0.1);
        }
        if (gamepad2.dpad_down) {
            kp = Math.min(0.1, kp - 0.1);
        }

        if (gamepad2.right_bumper) {
            maxSpeed += 0.05;
        }
        if (gamepad2.left_bumper) {
            maxSpeed = Math.max(0.05, maxSpeed - 0.05);
        }

        if (gamepad2.y) {
            tolerance += 2;
        }
        if (gamepad2.a) {
            tolerance = Math.max(5, tolerance - 2);
        }

        if (gamepad2.dpad_right) {
            tagId = 24; // Red
        }
        if (gamepad2.dpad_left) {
            tagId = 20; // Blue
        }

        // === DISPLAY CURRENT VALUES ===
        telemetry.addData("ALIGN TUNING", "Adjust with Gamepad2");
        telemetry.addData("KP", "%.2f  (DPAD ↑↓)", kp);
        telemetry.addData("Max Speed", "%.2f  (RB/LB)", maxSpeed);
        telemetry.addData("Tolerance", "%.0f px  (Y/A)", tolerance);
        telemetry.addData("Tag ID", "%d  (DPAD ←→)", tagId);
        telemetry.addData("Timeout", "%.1f sec", timeoutSec);
        telemetry.addData("", "");
        telemetry.addData("CONTROLS", "Gamepad1");
        telemetry.addData("Start", "A");
        telemetry.addData("Stop", "B");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === ADJUST TUNING VALUES ===
        if (gamepad2.dpad_up) {
            kp = Math.max(0.1, kp + 0.01);
        }
        if (gamepad2.dpad_down) {
            kp = Math.min(0.1, kp - 0.01);
        }

        if (gamepad2.right_bumper) {
            maxSpeed += 0.05;
        }
        if (gamepad2.left_bumper) {
            maxSpeed = Math.max(0.05, maxSpeed - 0.05);
        }

        if (gamepad2.y) {
            tolerance += 2;
        }
        if (gamepad2.a) {
            tolerance = Math.max(5, tolerance - 2);
        }

        if (gamepad2.dpad_right) {
            tagId = 24; // Red
        }
        if (gamepad2.dpad_left) {
            tagId = 20; // Blue
        }
        // === START ALIGNMENT ===
        if (gamepad1.a && !alignActive) {
            alignActive = true;
            telemetry.addData("ALIGN", "STARTED → Tag %d", tagId);
        }

        // === STOP ALIGNMENT ===
        if (gamepad1.b) {
            alignActive = false;
            aligner.stopAlignment();
            hardware.lf.setPower(0);
            hardware.rf.setPower(0);
            hardware.lr.setPower(0);
            hardware.rr.setPower(0);
            telemetry.addData("ALIGN", "STOPPED");
        }

        // === RUN ALIGNMENT ===
        if (alignActive) {
            // Inject current tuning values
            aligner.setKp(kp);
            aligner.setMaxSpeed(maxSpeed);
            aligner.setTolerance(tolerance);
            aligner.setTimeout(timeoutSec);

            aligner.alignRotationOnly(tagId);
        }

        // === TELEMETRY (always show) ===
        telemetry.addData("Status", alignActive ? "ALIGNING" : "IDLE");
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("KP", "%.2f", kp);
        telemetry.addData("Max Speed", "%.2f", maxSpeed);
        telemetry.addData("Tolerance", "%.0f", tolerance);
        telemetry.update();
    }

    @Override
    public void stop() {
        aligner.stopVision();
    }
}