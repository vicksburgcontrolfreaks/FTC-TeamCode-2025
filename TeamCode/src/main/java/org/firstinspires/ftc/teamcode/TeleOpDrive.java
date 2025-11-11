package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TestTeleOp", group = "TeleOp")
public class TeleOpDrive extends OpMode {

    // --------------------------------------------------------------------- //
    // --------------------------- HARDWARE ------------------------------ //
    // --------------------------------------------------------------------- //
    private RobotHardware hardware;
    private AlignAprilTag aligner;
    private Follower follower;

    // --------------------------------------------------------------------- //
    // -------------------------- 3-SHOT BURST -------------------------- //
    // --------------------------------------------------------------------- //
    private ThreeShots threeShots;

    // --------------------------------------------------------------------- //
    // --------------------------- CONTROLS ----------------------------- //
    // --------------------------------------------------------------------- //
    private double shooterTPS = 1600.0;
    private int tagId = 20;
    private final Timer debounceTimer = new Timer();
    private boolean lastA = false, lastB = false;
    private static final double DEBOUNCE_TIME = 0.2;

    // --------------------------------------------------------------------- //
    // --------------------------- DRIVE --------------------------------- //
    // --------------------------------------------------------------------- //
    private static final double DEADBAND = 0.1;
    private static final double SLOW_MODE_SPEED = 0.4;
    private boolean headingResetPressed = false;

    // --------------------------------------------------------------------- //
    // -------------------------- ALLIANCE ------------------------------- //
    // --------------------------------------------------------------------- //
    private boolean isRedAlliance = false;
    private double fieldForwardHeading = 0.0;

    // --------------------------------------------------------------------- //
    // -------------------------- AUTO HEADING --------------------------- //
    // --------------------------------------------------------------------- //
    private boolean headingInitialized = false;  // ← RUNS ONCE AFTER START

    // --------------------------------------------------------------------- //
    // ------------------------------ INIT ------------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        // Follower
        try {
            follower = Constants.createFollower(hardwareMap);
            // LOAD POSE FROM AUTON
            if (Math.abs(Constants.autonFinalHeading) > 0.01) {
                Pose savedPose = new Pose(
                        Constants.autonFinalX,
                        Constants.autonFinalY,
                        Constants.autonFinalHeading
                );
                follower.setPose(savedPose);

                telemetry.addData("POSE", "Loaded from Auton: X=%.1f Y=%.1f H=%.1f°",
                        savedPose.getX(), savedPose.getY(),
                        Math.toDegrees(savedPose.getHeading()));
            } else {
                follower.setPose(new Pose(0.0, 0.0, 0.0));
                telemetry.addData("POSE", "No auton data");
            }          hardware.addTelemetry("Status", "Follower initialized");
        } catch (Exception e) {
            hardware.addTelemetry("Error", "Follower failed: " + e.getMessage());
        }

        // Alignment
        aligner = new AlignAprilTag(hardware, follower, telemetry);
        aligner.setTelemetryEnabled(false);

        // 3-Shot
        threeShots = new ThreeShots(hardware);
        threeShots.setTelemetryEnabled(true);

        // Misc
        hardware.flipper.setPosition(0.0);
        debounceTimer.resetTimer();

        // Default alliance
        setAlliance(false);
    }

    // --------------------------------------------------------------------- //
    // --------------------------- INIT LOOP ----------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void init_loop() {
        if (gamepad2.dpad_up) setAlliance(false);
        if (gamepad2.dpad_down) setAlliance(true);

        telemetry.addData("Instructions", "Gamepad2: DPAD Up (Blue) / Down (Red)");
        telemetry.addData("Gamepad1", "Left Stick (Move), Right Stick (Rotate), Y (Align)");
        telemetry.addData("Gamepad2", "A (Collector), Bumpers (TPS), B (3-Shot Burst), X (Flipper)");
        telemetry.addData("Drive", "Left Bumper = Slow | Right Bumper = Reset Heading");
        telemetry.addData("Alliance", isRedAlliance ? "RED (0°)" : "BLUE (180°)");
        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Burst Ready", threeShots.isBusy() ? "BUSY" : "READY");
        aligner.updateTelemetry(tagId);
        telemetry.update();
    }

    // --------------------------------------------------------------------- //
    // -------------------------- ALLIANCE SET -------------------------- //
    // --------------------------------------------------------------------- //
    private void setAlliance(boolean red) {
        isRedAlliance = red;
        fieldForwardHeading = red ? 0.0 : Math.PI;
        tagId = red ? 24 : 20;
        RevBlinkinLedDriver.BlinkinPattern pattern = red ?
                RevBlinkinLedDriver.BlinkinPattern.RED :
                RevBlinkinLedDriver.BlinkinPattern.BLUE;
        hardware.leds.setPattern(pattern);
    }

    // --------------------------------------------------------------------- //
    // ------------------------------ LOOP ------------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void loop() {

        // --------------------------------------------------------------- //
        // ----------------------- LOCALIZER -------------------------- //
        // --------------------------------------------------------------- //
        follower.update();

        // --------------------------------------------------------------- //
        // ------------------- AUTO HEADING (ONCE) -------------------- //
        // --------------------------------------------------------------- //
        if (!headingInitialized) {
            AprilTagDetection tag = getBestTag(tagId);
            if (tag != null && tag.robotPose != null) {
                double tagYawRad = tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                double cameraYawRad = tagYawRad + Math.PI;  // BACK-FACING CAMERA

                // Normalize
                while (cameraYawRad > Math.PI) cameraYawRad -= 2 * Math.PI;
                while (cameraYawRad < -Math.PI) cameraYawRad += 2 * Math.PI;

                double robotHeading = fieldForwardHeading + cameraYawRad;

                // Final normalize
                while (robotHeading > Math.PI) robotHeading -= 2 * Math.PI;
                while (robotHeading < -Math.PI) robotHeading += 2 * Math.PI;

                follower.getPose().setHeading(robotHeading);
                telemetry.addData("AUTO HEADING", "Set to %.1f°", Math.toDegrees(robotHeading));

                headingInitialized = true;  // ← NEVER RUNS AGAIN
            }
        }

        // --------------------------------------------------------------- //
        // ------------------- FIELD-CENTRIC DRIVE -------------------- //
        // --------------------------------------------------------------- //
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRx = gamepad1.right_stick_x;

        double y = Math.abs(rawY) > DEADBAND ? rawY : 0.0;
        double x = Math.abs(rawX) > DEADBAND ? rawX : 0.0;
        double rx = Math.abs(rawRx) > DEADBAND ? rawRx : 0.0;

        boolean slowMode = gamepad1.left_bumper;
        double speedMul = slowMode ? SLOW_MODE_SPEED : 1.0;

        // Manual reset
        if (gamepad1.y && !headingResetPressed) {
            follower.getPose().setHeading(fieldForwardHeading);
            telemetry.addData("HEADING", "RESET TO %.0f°", Math.toDegrees(fieldForwardHeading));
        }
        headingResetPressed = gamepad1.y;

        // Field-centric math
        double botHeading = follower.getPose().getHeading() - fieldForwardHeading;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double scale = 1.0 / max;

        double lf = scale * (rotY + rotX + rx) * speedMul;
        double rf = scale * (rotY - rotX - rx) * speedMul;
        double lr = scale * (rotY - rotX + rx) * speedMul;
        double rr = scale * (rotY + rotX - rx) * speedMul;

        hardware.lf.setPower(lf);
        hardware.rf.setPower(rf);
        hardware.lr.setPower(lr);
        hardware.rr.setPower(rr);

        // LED
        hardware.leds.setPattern(slowMode ?
                RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE :
                (isRedAlliance ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.BLUE));

        // --------------------------------------------------------------- //
        // -------------------------- COLLECTOR ---------------------- //
        // --------------------------------------------------------------- //
// Y = CLEAR MISFEED (Collector reverse + Shooter reverse + Flipper reset)
        if (gamepad2.y) {
            // Interrupt 3-shot if running
            if (threeShots.isBusy()) {
                threeShots.interrupt();
            }

            // Collector reverse
            hardware.collector.setPower(-1.0);

            // Shooter reverse @ 0.5 power
            hardware.shooter.setPower(-0.5);

            // Reset flipper
            hardware.flipper.setPosition(0.0);

        } else if (gamepad2.a) {
            // A = Normal intake
            if (threeShots.isBusy()) {
                threeShots.interrupt();
            }
            hardware.collector.setPower(1.0);
            hardware.shooter.setPower(0.0);  // Normal: shooter off
            hardware.flipper.setPosition(0.0);

        } else {
            // No button → stop everything
            if (!threeShots.isBusy()) {
                hardware.collector.setPower(0.0);
                hardware.shooter.setPower(0.0);
            }
            hardware.flipper.setPosition(0.0);
        }
// Always encoder mode
//        hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --------------------------------------------------------------- //
        // ----------------------- 3-SHOT BURST ---------------------- //
        // --------------------------------------------------------------- //
        if (gamepad2.b && !lastB) {
            threeShots.start((int) shooterTPS);
        }
        lastB = gamepad2.b;
        threeShots.update(tagId);

        // --------------------------------------------------------------- //
        // ----------------------- MANUAL FLIPPER -------------------- //
        // --------------------------------------------------------------- //
        if (gamepad2.x && !threeShots.isBusy()) {
            hardware.flipper.setPosition(0.5);
        } else if (!threeShots.isBusy()) {
            hardware.flipper.setPosition(0.0);
        }

        // --------------------------------------------------------------- //
        // --------------------------- TELEMETRY --------------------- //
        // --------------------------------------------------------------- //
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Field Forward", "%.0f°", Math.toDegrees(fieldForwardHeading));
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("Shooter TPS Set", shooterTPS);
        telemetry.addData("Shooter Vel", "%.0f", hardware.shooter.getVelocity());
        telemetry.addData("Battery", "%.2fV", hardware.getBatteryVoltage());
        telemetry.addData("Burst", threeShots.isBusy() ? "BUSY" : "READY");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Drive Mode", slowMode ? "SLOW (LB)" : "NORMAL");
        telemetry.update();
    }

    // ----------------------------------------------------------------- //
    // -------------------------- HELPERS ---------------------------- //
    // ----------------------------------------------------------------- //
    private void stopDriveMotors() {
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

    private AprilTagDetection getBestTag(int desiredId) {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        AprilTagDetection best = null;
        double bestConfidence = 0;
        for (AprilTagDetection d : detections) {
            if (d.id == desiredId && d.robotPose != null) {
                double dist = d.robotPose.getPosition().z;
                double confidence = 1.0 / (dist + 1);
                if (confidence > bestConfidence) {
                    bestConfidence = confidence;
                    best = d;
                }
            }
        }
        return best;
    }
}