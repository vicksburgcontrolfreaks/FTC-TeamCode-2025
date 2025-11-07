package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TestTeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode {

    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;
    private ShooterSequence shooterSeq;

    private double collectorRPM = 0.0;
    private double shooterTPS = 1600.0;
    private boolean collectorOn = false;
    private int tagId = 20;

    private static final double TICKS_PER_REV = 28;
    private static final double COLLECTOR_GEAR_RATIO = 4.0;
    private static final double SHOOTER_GEAR_RATIO = 1.0;

    private Timer debounceTimer = new Timer();
    private boolean lastA = false, lastB = false;
    private static final double DEBOUNCE_TIME = 0.2;

    private static final int BURST_SIZE = 3;
    private int burstRemaining = 0;

    // === DRIVE ENHANCEMENTS ===
    private static final double DEADBAND = 0.1;
    private static final double SLOW_MODE_SPEED = 0.4;
    private boolean headingResetPressed = false;

    // === ALLIANCE & FIELD ORIENTATION ===
    private boolean isRedAlliance = false; // false = Blue, true = Red
    private double fieldForwardHeading = 0.0; // Red = 0°, Blue = 180°

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        try {
            follower = Constants.createFollower(hardwareMap);
            hardware.addTelemetry("Status", "Follower initialized");
        } catch (Exception e) {
            hardware.addTelemetry("Error", "Follower failed: " + e.getMessage());
        }

        shooter = new ShootAprilTag(hardware, follower, telemetry);
        shooter.setTelemetryEnabled(false);

        shooterSeq = new ShooterSequence(hardware, shooter);

        hardware.flipper.setPosition(0.0);
        debounceTimer.resetTimer();

        // Default to Blue
        setAlliance(false);
    }

    @Override
    public void init_loop() {
        if (gamepad2.dpad_up) {
            setAlliance(false); // Blue
        } else if (gamepad2.dpad_down) {
            setAlliance(true);  // Red
        }

        telemetry.addData("Instructions", "Gamepad2: DPAD Up (Blue) / Down (Red)");
        telemetry.addData("Gamepad1", "Left Stick (Move), Right Stick (Rotate), Y (Align)");
        telemetry.addData("Gamepad2", "A (Collector), Bumpers (RPM), B (3-Shot Burst), X (Flipper)");
        telemetry.addData("Drive", "Left Bumper = Slow Mode | Right Bumper = Reset Heading");
        telemetry.addData("Alliance", isRedAlliance ? "RED (0°)" : "BLUE (180°)");
        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Burst Ready", burstRemaining > 0 ? "YES" : "NO");

        shooter.updateTelemetry(tagId);
        telemetry.update();
    }

    private void setAlliance(boolean red) {
        isRedAlliance = red;
        fieldForwardHeading = red ? 0.0 : Math.PI; // 0° or 180°
        tagId = red ? 24 : 20;

        // Set normal LED color
        RevBlinkinLedDriver.BlinkinPattern normalPattern = red ?
                RevBlinkinLedDriver.BlinkinPattern.RED :
                RevBlinkinLedDriver.BlinkinPattern.BLUE;
        hardware.leds.setPattern(normalPattern);
    }

    @Override
    public void loop() {
        // === FIELD-CENTRIC DRIVE (ENHANCED + ALLIANCE) ===
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRx = gamepad1.right_stick_x;

        // === DEADBAND ===
        double y = Math.abs(rawY) > DEADBAND ? rawY : 0.0;
        double x = Math.abs(rawX) > DEADBAND ? rawX : 0.0;
        double rx = Math.abs(rawRx) > DEADBAND ? rawRx : 0.0;

        // === SLOW MODE (Left Bumper) ===
        boolean slowMode = gamepad1.left_bumper;
        double speedMultiplier = slowMode ? SLOW_MODE_SPEED : 1.0;

        // === HEADING RESET (Right Bumper) ===
        if (gamepad1.right_bumper && !headingResetPressed) {
            follower.getPose().setHeading(fieldForwardHeading);
            telemetry.addData("HEADING", "RESET TO %.0f°", Math.toDegrees(fieldForwardHeading));
        }
        headingResetPressed = gamepad1.right_bumper;

        // === FIELD-CENTRIC ROTATION (Relative to alliance forward) ===
        double botHeading = follower.getPose().getHeading() - fieldForwardHeading;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // === POWER NORMALIZATION ===
        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double scale = 1.0 / max;

        // === APPLY POWER ===
        double lfPower = scale * (rotY + rotX + rx) * speedMultiplier;
        double rfPower = scale * (rotY - rotX - rx) * speedMultiplier;
        double lrPower = scale * (rotY - rotX + rx) * speedMultiplier;
        double rrPower = scale * (rotY + rotX - rx) * speedMultiplier;

        hardware.lf.setPower(lfPower);
        hardware.rf.setPower(rfPower);
        hardware.lr.setPower(lrPower);
        hardware.rr.setPower(rrPower);

        // === DRIVE MODE LED VISUAL ===
        if (slowMode) {
            hardware.leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
        } else {
            // Normal mode: alliance color
            hardware.leds.setPattern(isRedAlliance ?
                    RevBlinkinLedDriver.BlinkinPattern.RED :
                    RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        // === AUTO-ALIGN ===
        if (gamepad1.y) {
            shooter.alignRotationOnly(tagId);
        }

        // === COLLECTOR ===
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

        if (collectorOn && !shooterSeq.isBusy()) {
            hardware.collector.setPower(-0.5);
        } else if (!shooterSeq.isBusy()) {
            hardware.collector.setPower(0.0);
        }

        // === SHOOTER TPS ===
        if (gamepad2.right_bumper) {
            shooterTPS = Math.min(shooterTPS + 20, 4000);
        } else if (gamepad2.left_bumper) {
            shooterTPS = Math.max(shooterTPS - 20, 0);
        }

        if (gamepad2.b && !lastB) {
            burstRemaining = 3;  // Queue 3 shots
        }
        lastB = gamepad2.b;

        // Start first shot if ready
        if (!shooterSeq.isBusy() && burstRemaining > 0) {
            shooterSeq.setBurstRemaining(burstRemaining - 1);  // Tell sequence
            shooterSeq.start(1600);  // 1600 ticks/sec
            burstRemaining--;
        }

        shooterSeq.update(tagId);

        // === MANUAL FLIPPER ===
        if (gamepad2.x && !shooterSeq.isBusy()) {
            hardware.flipper.setPosition(0.5);
        } else if (!shooterSeq.isBusy()) {
            hardware.flipper.setPosition(0.0);
        }

        // === LOCALIZER ===
        follower.update();

        // === TELEMETRY ===
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Field Forward", "%.0f°", Math.toDegrees(fieldForwardHeading));
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("Shooter TPS Set", shooterTPS);
        telemetry.addData("Shooter Vel", "%.0f", hardware.shooter.getVelocity());
        telemetry.addData("Battery", "%.2fV", hardware.getBatteryVoltage());
        telemetry.addData("Burst Remaining", burstRemaining);
//        telemetry.addData("LED", shooterSeq.getLEDPattern().toString().replace("BLINKIN_PATTERN_", ""));
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Drive Mode", slowMode ? "SLOW (LB)" : "NORMAL");
        telemetry.update();
    }
}