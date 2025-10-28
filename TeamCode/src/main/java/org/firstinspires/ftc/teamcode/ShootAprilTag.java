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
    private static final double TARGET_X = 320.0; // Center of 1280px image (adjust to 540 for 1080p)
    private static final double KP = 1.0; // Proportional gain
    private static final double TOLERANCE = 23.0; // Pixel tolerance
    private static final double MAX_ROT_SPEED = 0.2; // Max rotation power
    private static final double ALIGN_TIMEOUT = 3.0; // Seconds
    private Timer alignTimer;

    public ShootAprilTag(RobotHardware hardware, Follower follower, Telemetry telemetry) {
        this.hardware = hardware;
        this.follower = follower;
        this.telemetry = telemetry;
        this.alignTimer = new Timer();
    }

    public void alignRotationOnly(int tagId) {
        alignTimer.resetTimer(); // Reset timer at start

        double currentX = -1.0; // Default value if no tag detected
        double error = 0.0; // Default error
        double rotationPower = 0.0; // Default rotation power

        if (alignTimer.getElapsedTimeSeconds() > ALIGN_TIMEOUT) {
            stopMotors();
            telemetry.addData("Tag Status", "Alignment timed out");
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
                telemetry.addData("Tag Status", "Tag ID " + tagId + " not detected");
            } else {
                currentX = targetTag.center.x; // Pixel x-coordinate
                error = TARGET_X - currentX; // Positive error = tag is left, rotate CCW

                if (Math.abs(error) < TOLERANCE) {
                    stopMotors();
                    telemetry.addData("Tag Status", "Centered (X: " + currentX + ")");
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

        // Telemetry during alignment
        telemetry.addData("Tag X", currentX);
        telemetry.addData("Error", error);
        telemetry.addData("Rotation Power", rotationPower);
        telemetry.addData("Follower Status", follower != null ? "Initialized" : "Null");
        telemetry.addData("Timer", alignTimer.getElapsedTimeSeconds());
        telemetry.update();
    }

    public void updateTelemetry(int tagId) {
        double currentX = -1.0; // Default if no tag detected
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
            currentX = targetTag.center.x; // Pixel x-coordinate
            telemetry.addData("Tag Status", "Tag ID " + tagId + " detected");
        } else {
            telemetry.addData("Tag Status", "Tag ID " + tagId + " not detected");
        }

        // Continuous telemetry
        telemetry.addData("Tag X", currentX);
        telemetry.addData("Detections Count", detections != null ? detections.size() : 0);
        telemetry.addData("Camera State", hardware.visionPortal.getCameraState());
    }

    private void stopMotors() {
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

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
}