package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class ShootAprilTag {
    private final RobotHardware hardware;
    private final Follower follower;
    private final Telemetry telemetry;

    private boolean enableTelemetry = true;
    private boolean isAligning = false;

    // === TUNABLE ALIGNMENT PARAMETERS ===
    private double TARGET_X = 320.0;
    private double kp = 1.0;
    private double maxSpeed = 0.2;
    private double tolerance = 23.0;
    private double timeoutSec = 3.0;

    private final Timer alignTimer = new Timer();

    public ShootAprilTag(RobotHardware hardware, Follower follower, Telemetry telemetry) {
        this.hardware = hardware;
        this.follower = follower;
        this.telemetry = telemetry;
    }

    // === TELEMETRY CONTROL ===
    public void setTelemetryEnabled(boolean enabled) {
        this.enableTelemetry = enabled;
    }

    // === ALIGNMENT STATE ===
    public boolean isAligning() {
        return isAligning;
    }

    // === TUNABLE SETTERS ===
    public void setKp(double kp) { this.kp = kp; }
    public void setMaxSpeed(double maxSpeed) { this.maxSpeed = maxSpeed; }
    public void setTolerance(double tolerance) { this.tolerance = tolerance; }
    public void setTimeout(double timeoutSec) { this.timeoutSec = timeoutSec; }
    public void setTargetX(double targetX) { this.TARGET_X = targetX; }

    // === MAIN ALIGN METHOD ===
    public void alignRotationOnly(int tagId) {
        isAligning = true;
        alignTimer.resetTimer();
        stopMotors();  // Prevent drift

        // Timeout
        if (alignTimer.getElapsedTimeSeconds() > timeoutSec) {
            stopMotors();
            isAligning = false;
            if (enableTelemetry) telemetry.addData("Tag Status", "Timed out");
            return;
        }

        // Get target tag
        AprilTagDetection targetTag = getTargetTag(tagId);
        if (targetTag == null) {
            stopMotors();
            isAligning = false;
            if (enableTelemetry) telemetry.addData("Tag Status", "ID %d not found", tagId);
            return;
        }

        // Calculate error
        double currentX = targetTag.center.x;
        double error = TARGET_X - currentX;

        // Within tolerance
        if (Math.abs(error) < tolerance) {
            stopMotors();
            isAligning = false;
            if (enableTelemetry) telemetry.addData("Tag Status", "Centered (X: %.1f)", currentX);
            return;
        }

        // Apply rotation
        double rotationPower = kp * error;
        rotationPower = Math.max(-maxSpeed, Math.min(maxSpeed, rotationPower));

        hardware.lf.setPower( rotationPower);
        hardware.rf.setPower( rotationPower);
        hardware.lr.setPower(-rotationPower);
        hardware.rr.setPower(-rotationPower);

        // Telemetry
        if (enableTelemetry) {
            telemetry.addData("Tag Status", "Aligning...");
            telemetry.addData("Tag X", "%.1f", currentX);
            telemetry.addData("Error", "%.1f", error);
            telemetry.addData("Rot Power", "%.3f", rotationPower);
            telemetry.addData("Timer", "%.1f", alignTimer.getElapsedTimeSeconds());
        }
    }

    // === TELEMETRY-ONLY UPDATE ===
    public void updateTelemetry(int tagId) {
        AprilTagDetection targetTag = getTargetTag(tagId);
        double currentX = (targetTag != null) ? targetTag.center.x : -1.0;

        if (enableTelemetry) {
            if (targetTag != null) {
                telemetry.addData("Tag Status", "ID %d detected", tagId);
                telemetry.addData("Tag X", "%.1f", currentX);
            } else {
                telemetry.addData("Tag Status", "ID %d not detected", tagId);
                telemetry.addData("Tag X", "N/A");
            }
            List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
            telemetry.addData("Detections", detections != null ? detections.size() : 0);
        }
    }

    // === HELPER: Get target tag ===
    private AprilTagDetection getTargetTag(int tagId) {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        if (detections == null) return null;
        for (AprilTagDetection d : detections) {
            if (d.id == tagId) return d;
        }
        return null;
    }

    // === MOTOR CONTROL ===
    private void stopMotors() {
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

    // === VISION CONTROL ===
    public void stopVision() {
        if (hardware.visionPortal != null) {
            hardware.visionPortal.stopStreaming();
        }
    }

    public void startVision() {
        if (hardware.visionPortal != null) {
            hardware.visionPortal.resumeStreaming();
        }
    }

    // === CLEANUP ===
    public void stopAlignment() {
        stopMotors();
        isAligning = false;
    }
}