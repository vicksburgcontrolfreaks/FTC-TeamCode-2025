package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auton.AutonConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Red Drive", group = "TeleOp")
public class RedDrive extends OpMode {

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
    // -------------------------- AUTO-COLLECT --------------------------- //
    // --------------------------------------------------------------------- //
    private AutoCollect autoCollect;

    // --------------------------------------------------------------------- //
    // --------------------------- LED MANAGER -------------------------- //
    // --------------------------------------------------------------------- //
    private LEDManager ledManager;

    // --------------------------------------------------------------------- //
    // --------------------------- LIFT SYSTEM --------------------------- //
    // --------------------------------------------------------------------- //
    private LiftSystem liftSystem;

    // --------------------------------------------------------------------- //
    // --------------------------- CONTROLS ----------------------------- //
    // --------------------------------------------------------------------- //
    private int tagId = 24;
    private final Timer debounceTimer = new Timer();
    private boolean lastGP1A = false;
    private boolean lastGP2A = false, lastGP2B = false, lastGP2X = false, lastY = false;
    private boolean lastAutoCollectBusy = false;  // Track auto-collect state for LED flash
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

        // Follower - Start at RED loading zone
        try {
            follower = Constants.createFollower(hardwareMap);

            // Set start pose to redLongLoad
            follower.setPose(AutonConstants.redHandLoad);
            poseLoadedFromAuton = true;
            headingInitialized = true;

            telemetry.addData("POSE", "Starting at RED Long Load: X=%.1f Y=%.1f H=%.1f°",
                    AutonConstants.redHandLoad.getX(),
                    AutonConstants.redHandLoad.getY(),
                    Math.toDegrees(AutonConstants.redHandLoad.getHeading()));
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

        // Auto-Collect
        autoCollect = new AutoCollect(hardware);
        autoCollect.setTelemetryEnabled(true);

        // LED Manager
        ledManager = new LEDManager(hardware.leds, true);  // RED alliance

        // Lift System
        liftSystem = new LiftSystem(hardware.lWinch, hardware.rWinch, telemetry);

        // Loading Zone Command
        loadingZoneCmd = new LoadingZoneCommand(hardware, follower, telemetry, "RED");

        // Misc
        debounceTimer.resetTimer();

        // Set alliance to RED (fixed)
        setAlliance(true);
    }

    // --------------------------------------------------------------------- //
    // --------------------------- INIT LOOP ----------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void init_loop() {
        // Alliance is fixed to RED

        telemetry.addData("=== INSTRUCTIONS ===", "");
        telemetry.addData("Alliance", "RED (Fixed)");
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
        telemetry.addData("Left Stick Up", "Auto Lift to Limit");
        telemetry.addData("Left Stick Down", "Lower Lift (Variable Speed)");
        telemetry.addData("", "");
        telemetry.addData("=== STATUS ===", "");
        telemetry.addData("Alliance", "RED (Fixed - 0°)");
        telemetry.addData("Start Position", "Red Long Load");
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

        // Disable vision processing during teleop to reduce lag
        if (hardware.visionPortal != null &&
            hardware.visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            hardware.visionPortal.stopStreaming();
        }
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
        // DISABLED - Auto-drive command removed
        /*
        if (gamepad1.a && !lastGP1A) {
            if (loadingZoneCmd.isBusy()) {
                loadingZoneCmd.cancel();
                telemetry.addData("Loading Zone", "CANCELLED by driver");
            } else {
                loadingZoneCmd.start();
            }
        }
        lastGP1A = gamepad1.a;

        // Update loading zone command if running
        if (loadingZoneCmd.isBusy()) {
            loadingZoneCmd.update();
        }
        */

        // --------------------------------------------------------------- //
        // ----------------------- AUTO-COLLECT ------------------------ //
        // --------------------------------------------------------------- //
        // Don't allow manual collector control while loading zone command is active
        if (loadingZoneCmd.isBusy()) {
            // Loading zone command is controlling collector - skip manual controls
        }
        // Y = CLEAR JAM (Collector reverse + Shooter reverse + Ball servos reverse + Flipper reset)
        else if (gamepad2.y) {
            // Stop auto-collect if running
            if (autoCollect.isBusy()) {
                autoCollect.stop();
            }

            // Interrupt 3-shot if running
            if (threeShots.isBusy()) {
                threeShots.interrupt();
            }

            // Ensure collector is in correct mode
            if (hardware.collector.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Collector reverse
            hardware.collector.setPower(-1.0);

            // Shooter reverse @ 0.5 power
            hardware.shooter.setPower(-0.5);

            // Ball servos reverse to clear jam
            hardware.lBallServo.setPower(-1.0);
            hardware.rBallServo.setPower(-1.0);

            // Reset flipper
            hardware.flipper.setPosition(0.0);

        } else if (gamepad2.a && !lastGP2A) {
            // A = Toggle Auto-Collect (starts collection until 3 balls detected)
            if (autoCollect.isBusy()) {
                autoCollect.stop();
            } else {
                // Interrupt 3-shot if running
                if (threeShots.isBusy()) {
                    threeShots.interrupt();
                }
                autoCollect.start();
            }
        } else if (!gamepad2.y) {
            // No button → stop everything (if not in auto-collect or 3-shot)
            if (!autoCollect.isBusy() && !threeShots.isBusy()) {
                // Ensure collector is in correct mode before stopping
                if (hardware.collector.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    hardware.collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                hardware.collector.setPower(0.0);
                hardware.shooter.setPower(0.0);
                hardware.stopBallServos();
                hardware.flipper.setPosition(0.0);
            }
        }
        lastGP2A = gamepad2.a;

        // Update auto-collect (monitors sensors and stops when full)
        autoCollect.update();

        // Flash green when auto-collect finishes
        boolean autoCollectBusy = autoCollect.isBusy();
        if (lastAutoCollectBusy && !autoCollectBusy) {
            ledManager.flashGreen(5);  // Flicker green 5 times (~1.5 seconds)
        }
        lastAutoCollectBusy = autoCollectBusy;

        // --------------------------------------------------------------- //
        // ----------------------- 3-SHOT BURST ---------------------- //
        // --------------------------------------------------------------- //
        // X = Short shot
        if (gamepad2.x && !lastGP2X && !threeShots.isBusy()) {
            // Stop auto-collect if running
            if (autoCollect.isBusy()) {
                autoCollect.stop();
            }
            threeShots.startShortShot();
        }
        lastGP2X = gamepad2.x;

        // B = Long shot
        if (gamepad2.b && !lastGP2B && !threeShots.isBusy()) {
            // Stop auto-collect if running
            if (autoCollect.isBusy()) {
                autoCollect.stop();
            }
            threeShots.startLongShot();
        }
        lastGP2B = gamepad2.b;

        threeShots.update(tagId);

        // --------------------------------------------------------------- //
        // ----------------------- LIFT SYSTEM ------------------------ //
        // --------------------------------------------------------------- //
        liftSystem.update(gamepad2);

        // Update LED manager with lift limit status (keeps flashing until cleared)
        ledManager.setLiftLimit(liftSystem.isLimitReached());

        // --------------------------------------------------------------- //
        // ----------------------- MANUAL FLIPPER -------------------- //
        // --------------------------------------------------------------- //
        // Removed - no longer needed

        // --------------------------------------------------------------- //
        // --------------------------- TELEMETRY --------------------- //
        // --------------------------------------------------------------- //
        // Minimal telemetry to reduce lag - only show critical info
        telemetry.addData("Heading", "%.0f°", Math.toDegrees(follower.getPose().getHeading() + headingOffset));
        telemetry.addData("Shooter", "%.0f TPS", hardware.shooter.getVelocity());
        telemetry.addData("Balls", "%d/3", autoCollect.getBallCount());
        telemetry.addData("Battery", "%.1fV", hardware.getBatteryVoltage());
        if (liftSystem.isLimitReached()) {
            telemetry.addData("⚠ LIFT", "LIMIT REACHED");
        }
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