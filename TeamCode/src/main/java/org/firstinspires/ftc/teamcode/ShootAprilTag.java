package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public class ShootAprilTag {
    private RobotHardware hardware;
    private Follower follower;
    private Telemetry telemetry;
    private boolean enableTelemetry = true;          // <<< NEW toggle
    private static final double TARGET_X = 320.0;
    private static final double KP = 1.0;
    private static final double TOLERANCE = 23.0;
    private static final double MAX_ROT_SPEED = 0.2;
    private static final double ALIGN_TIMEOUT = 3.0;
    private Timer alignTimer;

    public ShootAprilTag(RobotHardware hardware, Follower follower, Telemetry telemetry) {
        this.hardware = hardware;
        this.follower = follower;
        this.telemetry = telemetry;
        this.alignTimer = new Timer();
    }

    // ---- optional setter -------------------------------------------------
    public void setTelemetryEnabled(boolean enabled) {
        this.enableTelemetry = enabled;
    }
    // ---------------------------------------------------------------------

    public void alignRotationOnly(int tagId) {
        alignTimer.resetTimer();
        double currentX = -1.0;
        double error = 0.0;
        double rotationPower = 0.0;

        if (alignTimer.getElapsedTimeSeconds() > ALIGN_TIMEOUT) {
            stopMotors();
            if (enableTelemetry) telemetry.addData("Tag Status", "Alignment timed out");
        } else {
            List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
            AprilTagDetection targetTag = null;
            for (AprilTagDetection detection : detections) {
                if (detection.id == tagId) {
                    targetTag = detection;
                    break;
                }
            }

            if (targetTag == null) {
                stopMotors();
                if (enableTelemetry) telemetry.addData("Tag Status", "Tag ID " + tagId + " not detected");
            } else {
                currentX = targetTag.center.x;
                error = TARGET_X - currentX;

                if (Math.abs(error) < TOLERANCE) {
                    stopMotors();
                    if (enableTelemetry) telemetry.addData("Tag Status", "Centered (X: " + currentX + ")");
                    alignTimer.resetTimer();
                } else {
                    rotationPower = KP * error;
                    rotationPower = Math.max(-MAX_ROT_SPEED, Math.min(MAX_ROT_SPEED, rotationPower));
                    double heading = (follower != null && follower.getPose() != null) ? follower.getPose().getHeading() : 0.0;
                    hardware.lf.setPower(-rotationPower);
                    hardware.rf.setPower(rotationPower);
                    hardware.lr.setPower(-rotationPower);
                    hardware.rr.setPower(rotationPower);
                }
            }
        }

        // ---- telemetry block ------------------------------------------------
        if (enableTelemetry) {
            telemetry.addData("Tag X", currentX);
            telemetry.addData("Error", error);
            telemetry.addData("Rotation Power", rotationPower);
            telemetry.addData("Follower Status", follower != null ? "Initialized" : "Null");
            telemetry.addData("Timer", alignTimer.getElapsedTimeSeconds());
            telemetry.update();
        }
        // --------------------------------------------------------------------
    }

    public void updateTelemetry(int tagId) {
        double currentX = -1.0;
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        AprilTagDetection targetTag = null;

        if (detections != null) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == tagId) {
                    targetTag = detection;
                    break;
                }
            }
        }

        if (targetTag != null) {
            currentX = targetTag.center.x;
            if (enableTelemetry) telemetry.addData("Tag Status", "Tag ID " + tagId + " detected");
        } else {
            if (enableTelemetry) telemetry.addData("Tag Status", "Tag ID " + tagId + " not detected");
        }

        // ---- telemetry block ------------------------------------------------
        if (enableTelemetry) {
            telemetry.addData("Tag X", currentX);
            telemetry.addData("Detections Count", detections != null ? detections.size() : 0);
            telemetry.addData("Camera State", hardware.visionPortal.getCameraState());
            // note: update() is usually called outside this method
        }
        // --------------------------------------------------------------------
    }

    private void stopMotors() {
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

    public void stopVision() {
        if (hardware.visionPortal != null) hardware.visionPortal.stopStreaming();
    }

    public void startVision() {
        if (hardware.visionPortal != null) hardware.visionPortal.resumeStreaming();
    }
}