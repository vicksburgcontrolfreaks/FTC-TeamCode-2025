package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp(name = "TestTeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode {
    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;
    private double collectorRPM = 0.0;
    private double shooterRPM = 0.0;
    private boolean collectorOn = false;
    private boolean shooterOn = false;
    private int tagId = 20; // Default to Blue alliance
    private static final double TICKS_PER_REV = 28;
    private static final double COLLECTOR_GEAR_RATIO = 4.0;
    private static final double SHOOTER_GEAR_RATIO = 1.0;
    private Timer debounceTimer = new Timer();
    private boolean lastA = false, lastB = false;
    private static final double DEBOUNCE_TIME = 0.2;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry); // Pass telemetry
        try {
            follower = Constants.createFollower(hardwareMap);
            hardware.addTelemetry("Status", "Follower initialized");
        } catch (Exception e) {
            hardware.addTelemetry("Error", "Follower failed: " + e.getMessage());
        }
        shooter = new ShootAprilTag(hardware, follower, telemetry);
        hardware.flipper.setPosition(0.0);
        debounceTimer.resetTimer();
    }

    @Override
    public void init_loop() {
        if (gamepad2.dpad_up) {
            tagId = 20;
        } else if (gamepad2.dpad_down) {
            tagId = 24;
        }
        telemetry.addData("Instructions", "Gamepad2: DPAD Up (Blue) or Down (Red)");
        telemetry.addData("Gamepad1", "Left Stick (Move/Strafe), Right Stick (Rotate), Y (Auto-Align)");
        telemetry.addData("Gamepad2", "DPAD Up/Down (Collector RPM +/-), A (Collector On/Off)");
        telemetry.addData("Gamepad2 Extra", "Right/Left Bumper (Shooter RPM +/-), B (Shooter On/Off), X (Flipper)");
        telemetry.addData("Alliance", tagId);
        // Continuous AprilTag telemetry
        shooter.updateTelemetry(tagId);
        telemetry.update();
    }

    @Override
    public void loop() {
        // Field-centric drive
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double heading = follower.getPose().getHeading();
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);
        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        hardware.lf.setPower((rotY + rotX + rx) / den);
        hardware.rf.setPower((rotY - rotX - rx) / den);
        hardware.lr.setPower((rotY - rotX + rx) / den);
        hardware.rr.setPower((rotY + rotX - rx) / den);

        // Continuous AprilTag telemetry
        shooter.updateTelemetry(tagId);

        // Auto-align rotation
        if (gamepad1.y) {
            shooter.alignRotationOnly(tagId);
        }

        // Collector controls
        if (gamepad2.dpad_up) {
            collectorRPM += 1;
        } else if (gamepad2.dpad_down) {
            collectorRPM -= 1;
        }
        if (gamepad2.a && !lastA && debounceTimer.getElapsedTimeSeconds() > DEBOUNCE_TIME) {
            collectorOn = !collectorOn;
            debounceTimer.resetTimer();
        }
        lastA = gamepad2.a;
        if (collectorOn) {
            double collectorTicksPerSec = collectorRPM * TICKS_PER_REV * COLLECTOR_GEAR_RATIO / 60.0;
            hardware.collector.setVelocity(collectorTicksPerSec);
        } else {
            hardware.collector.setPower(0.0);
        }

        // Shooter controls
        if (gamepad2.right_bumper) {
            shooterRPM = Math.min(shooterRPM + 100, 3000);
        } else if (gamepad2.left_bumper) {
            shooterRPM = Math.max(shooterRPM - 100, 0);
        }
        if (gamepad2.b && !lastB && debounceTimer.getElapsedTimeSeconds() > DEBOUNCE_TIME) {
            shooterOn = !shooterOn;
            debounceTimer.resetTimer();
        }
        lastB = gamepad2.b;
        if (shooterOn) {
            double shooterTicksPerSec = shooterRPM * TICKS_PER_REV * SHOOTER_GEAR_RATIO / 60.0;
            hardware.shooter.setVelocity(shooterTicksPerSec);
        } else {
            hardware.shooter.setPower(0.0);
        }

        // Flipper servo
        if (gamepad2.x) {
            hardware.flipper.setPosition(0.5);
        } else {
            hardware.flipper.setPosition(0.0);
        }

        // Update localizer
        follower.update();

        // Telemetry
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("Collector RPM", collectorRPM + " (On: " + collectorOn + ")");
        telemetry.addData("Collector Actual RPM",
                (hardware.collector.getVelocity() / (TICKS_PER_REV * COLLECTOR_GEAR_RATIO)) * 60.0);
        telemetry.addData("Shooter RPM", shooterRPM + " (On: " + shooterOn + ")");
        telemetry.addData("Shooter Actual RPM",
                (hardware.shooter.getVelocity() / (TICKS_PER_REV * SHOOTER_GEAR_RATIO)) * 60.0);
        telemetry.addData("Servo Pos", hardware.flipper.getPosition());
        telemetry.addData("Magazine Full", hardware.isMagazineFull());
        telemetry.update();
    }
}