package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class ShootAprilTag {
    private RobotHardware hardware;
    private double SHOOTER_VELOCITY_RPM = 2000; // RPM, tune with testing
    private double COLLECTOR_VELOCITY_RPM = 1000; // RPM for flywheel mode
    private double FLIP_POSITION_LAUNCH = 0.5; // Servo pos for launch
    private double FLIP_POSITION_RESET = 0.0; // Reset pos
    private double REVERSE_TIME = 0.5; // Sec to reverse collector
    private static final double TICKS_PER_REV = 2000; // From Constants (GoBilda motor)

    public ShootAprilTag(RobotHardware hardware) {
        this.hardware = hardware;
    }

    public void shoot(int tagId) {
        // Step 1: Align to AprilTag (ID 20 or 24, center in view)
        alignToTag(tagId);

        // Step 2: Shooting sequence for 3 balls
        // Spin shooter (velocity based on distance - TBD)
        double distance = getDistanceToTag(tagId); // Pass tagId
        SHOOTER_VELOCITY_RPM = calculateVelocity(distance); // Tune function
        double shooterTicksPerSec = SHOOTER_VELOCITY_RPM * TICKS_PER_REV / 60.0;
        hardware.shooter.setVelocity(shooterTicksPerSec);

        for (int i = 0; i < 3; i++) {
            // Spin collector as flywheel
            double collectorTicksPerSec = COLLECTOR_VELOCITY_RPM * TICKS_PER_REV / 60.0;
            hardware.collector.setVelocity(collectorTicksPerSec);

            // Flip to launch
            hardware.flipper.setPosition(FLIP_POSITION_LAUNCH);
            waitSec(0.5); // Time to launch

            // Reverse collector briefly
            hardware.collector.setPower(-0.3); // Reverse
            waitSec(REVERSE_TIME);
            hardware.collector.setPower(0);

            // Reset flipper
            hardware.flipper.setPosition(FLIP_POSITION_RESET);
            waitSec(0.2); // Reset time
        }

        // Stop motors
        hardware.shooter.setPower(0);
        hardware.collector.setPower(0);
    }

    private void alignToTag(int tagId) {
        // Simple PID to center tag (tune gains)
        double kp = 0.01, ki = 0.0, kd = 0.0;
        double prevErrorX = 0, integralX = 0;
        double prevErrorY = 0, integralY = 0;
        double prevErrorYaw = 0, integralYaw = 0;
        Timer timeout = new Timer();
        double TIMEOUT_SEC = 5.0; // Stop after 5 sec if no tag

        while (timeout.getElapsedTimeSeconds() < TIMEOUT_SEC) {
            AprilTagDetection detection = getDetection(tagId);
            if (detection == null) {
                hardware.visionPortal.getCameraState(); // Ensure streaming
                continue;
            }

            // Errors: Aim for x=0, y=0, yaw=0 (centered)
            double errorX = detection.ftcPose.x;
            double errorY = detection.ftcPose.y;
            double errorYaw = detection.ftcPose.yaw;

            integralX += errorX;
            integralY += errorY;
            integralYaw += errorYaw;

            double derivX = errorX - prevErrorX;
            double derivY = errorY - prevErrorY;
            double derivYaw = errorYaw - prevErrorYaw;

            double strafe = kp * errorX + ki * integralX + kd * derivX;
            double forward = kp * errorY + ki * integralY + kd * derivY;
            double turn = kp * errorYaw + ki * integralYaw + kd * derivYaw;

            // Drive powers (match Constants motor names)
            hardware.lf.setPower(forward + strafe + turn);
            hardware.rf.setPower(forward - strafe - turn);
            hardware.lr.setPower(forward - strafe + turn);
            hardware.rr.setPower(forward + strafe - turn);

            prevErrorX = errorX;
            prevErrorY = errorY;
            prevErrorYaw = errorYaw;

            if (Math.abs(errorX) < 1 && Math.abs(errorY) < 1 && Math.abs(errorYaw) < 5) break; // Threshold
        }

        // Stop drive
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }

    private AprilTagDetection getDetection(int tagId) {
        for (AprilTagDetection detection : hardware.aprilTagProcessor.getDetections()) {
            if (detection.id == tagId) return detection;
        }
        return null;
    }

    private double getDistanceToTag(int tagId) {
        AprilTagDetection detection = getDetection(tagId);
        return detection != null ? detection.ftcPose.range : 0; // Inches
    }

    private double calculateVelocity(double distance) {
        // TBD with testing, e.g., linear: 1500 + 50 * distance
        return 1500 + 50 * distance; // RPM
    }

    private void waitSec(double sec) {
        long ms = (long) (sec * 1000);
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            // Ignore
        }
    }
}