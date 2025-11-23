package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Drive TeleOp", group = "TeleOp")
public class TeleOpDrive extends OpMode {

    // --------------------------------------------------------------------- //
    // --------------------------- HARDWARE ------------------------------ //
    // --------------------------------------------------------------------- //
    private RobotHardware hardware;
    private AlignAprilTag aligner;
    private Follower follower;
    private LoadingZoneCommand loadingZoneCmd;


    // --------------------------------------------------------------------- //
    // -------------------------- 3-SHOT BURST -------------------------- //
    // --------------------------------------------------------------------- //
    private ThreeShots threeShots;

    // --------------------------------------------------------------------- //
    // --------------------------- LED MANAGER -------------------------- //
    // --------------------------------------------------------------------- //
    private LEDManager ledManager;

    // --------------------------------------------------------------------- //
    // --------------------------- CONTROLS ----------------------------- //
    // --------------------------------------------------------------------- //
    private double longShotTPS = 1300.0;
    private double shortShotTPS = 1000.0;
    private int tagId = 20;
    private final Timer debounceTimer = new Timer();
    private boolean lastA = false, lastB = false, lastX = false, lastY = false;
    private static final double DEBOUNCE_TIME = 0.2;

    // --------------------------------------------------------------------- //
    // --------------------------- DRIVE --------------------------------- //
    // --------------------------------------------------------------------- //
    private static final double DEADBAND = 0.1;
    private static final double SLOW_MODE_SPEED = 0.4;
    private double headingOffset = 0.0;  // Manual heading adjustment

    // --------------------------------------------------------------------- //
    // -------------------------- ALLIANCE ------------------------------- //
    // --------------------------------------------------------------------- //
    private boolean isRedAlliance = false;
    private double fieldForwardHeading = 0.0;

    // --------------------------------------------------------------------- //
    // -------------------------- AUTO HEADING --------------------------- //
    // --------------------------------------------------------------------- //
    private boolean headingInitialized = false;
    private boolean poseLoadedFromAuton = false;

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
                poseLoadedFromAuton = true;
                headingInitialized = true;  // Skip AprilTag heading if we have auton data

                telemetry.addData("POSE", "Loaded from Auton: X=%.1f Y=%.1f H=%.1f°",
                        savedPose.getX(), savedPose.getY(),
                        Math.toDegrees(savedPose.getHeading()));
            } else {
                follower.setPose(new Pose(0.0, 0.0, 0.0));
                telemetry.addData("POSE", "No auton data - will use AprilTag");
            }
            hardware.addTelemetry("Status", "Follower initialized");
        } catch (Exception e) {
            hardware.addTelemetry("Error", "Follower failed: " + e.getMessage());
        }

        // Alignment
        aligner = new AlignAprilTag(hardware, follower, telemetry);
        aligner.setTelemetryEnabled(false);

        // 3-Shot
        threeShots = new ThreeShots(hardware);
        threeShots.setTelemetryEnabled(true);

        // LED Manager
        ledManager = new LEDManager(hardware.leds, false);  // Default blue, will be updated in init_loop

        // Loading Zone Command
        loadingZoneCmd = new LoadingZoneCommand(hardware, follower, telemetry, "BLUE");

        // Misc
        debounceTimer.resetTimer();

        // Default alliance (will be overridden in init_loop)
        setAlliance(false);
    }

    // --------------------------------------------------------------------- //
    // --------------------------- INIT LOOP ----------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void init_loop() {
        if (gamepad2.dpad_up) setAlliance(false);
        if (gamepad2.dpad_down) setAlliance(true);

        telemetry.addData("=== INSTRUCTIONS ===", "");
        telemetry.addData("Alliance Select", "Gamepad2 DPAD Up (Blue) / Down (Red)");
        telemetry.addData("", "");
        telemetry.addData("=== GAMEPAD 1 (DRIVE) ===", "");
        telemetry.addData("Left Stick", "Move");
        telemetry.addData("Right Stick", "Rotate");
        telemetry.addData("Left Bumper", "Slow Mode");
        telemetry.addData("A Button", "Auto-Drive to Loading Zone (press again to cancel)");
        telemetry.addData("Y Button", "Reset Heading (Flash Green x3)");
        telemetry.addData("", "");
        telemetry.addData("=== GAMEPAD 2 (MECHANISMS) ===", "");
        telemetry.addData("A", "3-Shot Burst @ 1300 TPS");
        telemetry.addData("B", "3-Shot Burst @ 1600 TPS");
        telemetry.addData("X", "Manual Flipper");
        telemetry.addData("Y", "Clear Misfeed (Reverse All)");
        telemetry.addData("", "");
        telemetry.addData("=== STATUS ===", "");
        telemetry.addData("Alliance", isRedAlliance ? "RED (0°)" : "BLUE (180°)");
        telemetry.addData("Long Shot TPS", longShotTPS);
        telemetry.addData("Burst Ready", threeShots.isBusy() ? "BUSY" : "READY");
        telemetry.addData("Pose Source", poseLoadedFromAuton ? "AUTON" : "DEFAULT");
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

        // Update LED Manager
        ledManager = new LEDManager(hardware.leds, red);

        // Update Loading Zone Command with correct alliance
        if (follower != null) {
            loadingZoneCmd = new LoadingZoneCommand(hardware, follower, telemetry, red ? "RED" : "BLUE");
        }

        // If we didn't load from auton, set heading to alliance forward
        if (!poseLoadedFromAuton && follower != null) {
            follower.getPose().setHeading(fieldForwardHeading);
            headingOffset = 0.0;  // Reset offset
            headingInitialized = true;
        }
    }

    // --------------------------------------------------------------------- //
    // ------------------------------ LOOP ------------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void start() {
        hardware.flipper.setPosition(0.0);
    }
    public void loop() {

        // --------------------------------------------------------------- //
        // ------------------- LED MANAGER UPDATE --------------------- //
        // --------------------------------------------------------------- //
        ledManager.update();

        // --------------------------------------------------------------- //
        // ------------------- SAFETY CHECK --------------------------- //
        // --------------------------------------------------------------- //
        // Ensure collector is in correct mode when not in 3-shot sequence
        if (!threeShots.isBusy() &&
                hardware.collector.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
            hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.collector.setPower(0.0);
            telemetry.addData("SAFETY", "Collector mode reset");
        }

        // --------------------------------------------------------------- //
        // ----------------------- LOCALIZER -------------------------- //
        // --------------------------------------------------------------- //
        follower.update();

        // --------------------------------------------------------------- //
        // ------------------- AUTO HEADING (ONCE) -------------------- //
        // --------------------------------------------------------------- //
        // Only run if we didn't load pose from auton
        if (!headingInitialized && !poseLoadedFromAuton) {
            AprilTagDetection tag = getBestTag(tagId);
            if (tag != null && tag.robotPose != null) {
                double tagYawRad = tag.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
                double cameraYawRad = tagYawRad + Math.PI;  // BACK-FACING CAMERA

                // Normalize camera yaw
                while (cameraYawRad > Math.PI) cameraYawRad -= 2 * Math.PI;
                while (cameraYawRad < -Math.PI) cameraYawRad += 2 * Math.PI;

                // Robot heading = alliance forward + camera offset
                double robotHeading = fieldForwardHeading + cameraYawRad;

                // Normalize robot heading
                while (robotHeading > Math.PI) robotHeading -= 2 * Math.PI;
                while (robotHeading < -Math.PI) robotHeading += 2 * Math.PI;

                follower.getPose().setHeading(robotHeading);
                telemetry.addData("AUTO HEADING", "Set to %.1f° via AprilTag", Math.toDegrees(robotHeading));

                headingInitialized = true;
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

        // Manual heading reset (Y BUTTON)
        if (gamepad1.y && !lastY) {
            // Calculate what the current heading error is
            double currentHeading = follower.getPose().getHeading();
            // Set offset so that current robot orientation becomes the field forward
            headingOffset = fieldForwardHeading - currentHeading;

            ledManager.flashGreen(3);  // Flash green 3 times
            telemetry.addData("HEADING", "RESET! Current=%.1f° Offset=%.1f°",
                    Math.toDegrees(currentHeading), Math.toDegrees(headingOffset));
        }
        lastY = gamepad1.y;

        // Field-centric transformation
        // Apply the heading offset to correct for manual resets
        double botHeading = (follower.getPose().getHeading() + headingOffset) - fieldForwardHeading;

        // Rotate stick inputs by robot heading
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Calculate motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        double lf = (rotY + rotX + rx) / denominator * speedMul;
        double rf = (rotY - rotX - rx) / denominator * speedMul;
        double lr = (rotY - rotX + rx) / denominator * speedMul;
        double rr = (rotY + rotX - rx) / denominator * speedMul;

        // Only set drive powers if loading zone command is not busy
        if (!loadingZoneCmd.isBusy()) {
            hardware.lf.setPower(lf);
            hardware.rf.setPower(rf);
            hardware.lr.setPower(lr);
            hardware.rr.setPower(rr);
        }

        // LED handled by LEDManager

        // --------------------------------------------------------------- //
        // ------------------- LOADING ZONE COMMAND ------------------ //
        // --------------------------------------------------------------- //
        // A = Auto-drive to loading zone (or cancel if already running)
        if (gamepad1.a && !lastA) {
            if (loadingZoneCmd.isBusy()) {
                loadingZoneCmd.cancel();
                telemetry.addData("Loading Zone", "CANCELLED by driver");
            } else {
                loadingZoneCmd.start();
            }
        }
        lastA = gamepad1.a;

        // Update loading zone command if running
        if (loadingZoneCmd.isBusy()) {
            loadingZoneCmd.update();
        }

        // --------------------------------------------------------------- //
        // -------------------------- COLLECTOR ---------------------- //
        // --------------------------------------------------------------- //
        // Don't allow manual collector control while loading zone command is active
        if (loadingZoneCmd.isBusy()) {
            // Loading zone command is controlling collector - skip manual controls
        }
        // Y = CLEAR JAM (Collector reverse + Shooter reverse + Flipper reset)
        else if (gamepad2.y) {
            // Interrupt 3-shot if running
            if (threeShots.isBusy()) {
                threeShots.interrupt();
            }

            // Ensure collector is in correct mode
            if (hardware.collector.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Collector reverse
            hardware.collector.setPower(-0.2);

            // Shooter reverse @ 0.5 power
            hardware.shooter.setPower(-0.5);

            // Reset flipper
            hardware.flipper.setPosition(0.0);

        } else if (gamepad2.a) {
            // A = Normal intake
            if (threeShots.isBusy()) {
                threeShots.interrupt();
            }

            // Ensure collector is in correct mode
            if (hardware.collector.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            hardware.collector.setPower(1.0);
            hardware.shooter.setPower(-0.10);  // Slight reverse to prevent jamming
            hardware.flipper.setPosition(0.0);

        } else {
            // No button → stop everything (if not in 3-shot)
            if (!threeShots.isBusy()) {
                // Ensure collector is in correct mode before stopping
                if (hardware.collector.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                hardware.collector.setPower(0.0);
                hardware.shooter.setPower(0.0);
                hardware.flipper.setPosition(0.0);
            }
        }

        // --------------------------------------------------------------- //
        // ----------------------- 3-SHOT BURST ---------------------- //
        // --------------------------------------------------------------- //
        // X = Short shot (1000 TPS)
        if (gamepad2.x && !lastX && !threeShots.isBusy()) {
            threeShots.start((int) shortShotTPS);
        }
        lastX = gamepad2.x;

        // B = Long shot (1300 TPS)
        if (gamepad2.b && !lastB && !threeShots.isBusy()) {
            threeShots.start((int) longShotTPS);
        }
        lastB = gamepad2.b;

        threeShots.update(tagId);

        // --------------------------------------------------------------- //
        // ----------------------- MANUAL FLIPPER -------------------- //
        // --------------------------------------------------------------- //
        // Removed - no longer needed

        // --------------------------------------------------------------- //
        // --------------------------- TELEMETRY --------------------- //
        // --------------------------------------------------------------- //
        telemetry.addData("=== ALLIANCE ===", "");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Field Forward", "%.0f°", Math.toDegrees(fieldForwardHeading));
        telemetry.addData("Tag ID", tagId);

        telemetry.addData("", "");
        telemetry.addData("=== POSE ===", "");
        telemetry.addData("X", "%.1f", follower.getPose().getX());
        telemetry.addData("Y", "%.1f", follower.getPose().getY());
        telemetry.addData("Raw Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Heading Offset", "%.1f°", Math.toDegrees(headingOffset));
        telemetry.addData("Effective Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading() + headingOffset));
        telemetry.addData("Relative to Field", "%.1f°", Math.toDegrees((follower.getPose().getHeading() + headingOffset) - fieldForwardHeading));

        telemetry.addData("", "");
        telemetry.addData("=== SHOOTER ===", "");
        telemetry.addData("Long Shot TPS", longShotTPS);
        telemetry.addData("Short Shot TPS", shortShotTPS);
        telemetry.addData("Current Velocity", "%.0f", hardware.shooter.getVelocity());
        telemetry.addData("Burst Status", threeShots.isBusy() ? "BUSY" : "READY");

        telemetry.addData("", "");
        telemetry.addData("=== COLLECTOR ===", "");
        telemetry.addData("Mode", hardware.collector.getMode());
        telemetry.addData("Power", "%.2f", hardware.collector.getPower());
        telemetry.addData("Motor Busy", hardware.collector.isBusy());
        telemetry.addData("Position", hardware.collector.getCurrentPosition());

        telemetry.addData("", "");
        telemetry.addData("=== LOADING ZONE ===", "");
        telemetry.addData("Status", loadingZoneCmd.isBusy() ? "ACTIVE (Press A to cancel)" : "READY (Press A to start)");
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");

        telemetry.addData("", "");
        telemetry.addData("=== SYSTEM ===", "");
        telemetry.addData("Drive Mode", slowMode ? "SLOW (LB)" : "NORMAL");
        telemetry.addData("Battery", "%.2fV", hardware.getBatteryVoltage());

        telemetry.update();
    }

    // ----------------------------------------------------------------- //
    // -------------------------- HELPERS ---------------------------- //
    // ----------------------------------------------------------------- //
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