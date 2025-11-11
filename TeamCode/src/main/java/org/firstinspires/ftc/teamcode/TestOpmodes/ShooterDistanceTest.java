package org.firstinspires.ftc.teamcode.TestOpmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TEST: Shooter Distance + Pose", group = "Tests")
public class ShooterDistanceTest extends OpMode {

    private RobotHardware hardware;

    // Alliance tags
    private static final int BLUE_TAG = 20;
    private static final int RED_TAG  = 24;

    // Alliance selection (DPAD in init_loop)
    private boolean isRedAlliance = false;   // false = Blue
    private int      targetTag     = BLUE_TAG;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        telemetry.addData("STATUS", "Shooter Distance + Pose Test Ready");
        telemetry.addData("GOAL", "Face Tag 20/24 → Check Distance + X/Y/Heading");
        telemetry.addData("Camera", "Back-facing, 78° FOV");
        telemetry.addData("Alliance", "DPAD Up = Blue, Down = Red");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        if (gamepad2.dpad_up)  isRedAlliance = false;
        if (gamepad2.dpad_down) isRedAlliance = true;

        targetTag = isRedAlliance ? RED_TAG : BLUE_TAG;

        telemetry.addData("Alliance", isRedAlliance ? "RED (Tag 24)" : "BLUE (Tag 20)");
        telemetry.update();
    }

    @Override
    public void loop() {
        // SINGLE TAG (distance + velocity)
        AprilTagDetection singleTag = getBestTag(targetTag);

        // DUAL TAG POSE (localization)
        Pose dualPose = getPoseFromDualTags();

        // ---------- SINGLE TAG ----------
        if (singleTag != null && singleTag.robotPose != null) {
            double distance = singleTag.robotPose.getPosition().z;

            telemetry.addData("SINGLE TAG", "ID %d", singleTag.id);
            telemetry.addData("  Distance (Z)", "%.1f in", distance);
            telemetry.addData("  Suggested Velocity", getVelocity(distance));
        } else {
            telemetry.addData("SINGLE TAG", "NOT FOUND");
        }

        // ---------- DUAL TAG POSE ----------
        if (dualPose != null) {
            telemetry.addData("DUAL TAG POSE", "");
            telemetry.addData("  X", "%.1f in", dualPose.getX());
            telemetry.addData("  Y", "%.1f in", dualPose.getY());
            telemetry.addData("  Heading", "%.1f°", Math.toDegrees(dualPose.getHeading()));
            telemetry.addData("  Accuracy", "High (2 tags)");
        } else {
            telemetry.addData("DUAL TAG POSE", "Need both 20 and 24");
        }

        telemetry.addData("Detections", hardware.aprilTagProcessor.getDetections().size());
        telemetry.update();
    }

    // -----------------------------------------------------------------
    // -------------------------- HELPERS -----------------------------
    // -----------------------------------------------------------------

    /** Return the closest tag with the desired ID (20 or 24 only) */
    private AprilTagDetection getBestTag(int desiredId) {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        AprilTagDetection best = null;
        double bestDist = Double.MAX_VALUE;

        for (AprilTagDetection d : detections) {
            if (d.id == desiredId && d.robotPose != null) {
                double dist = d.robotPose.getPosition().z;
                if (dist < bestDist) {
                    bestDist = dist;
                    best = d;
                }
            }
        }
        return best;
    }

    /** Triangulate pose using BOTH tag 20 and tag 24 */
    private Pose getPoseFromDualTags() {
        AprilTagDetection tag20 = getBestTag(BLUE_TAG);
        AprilTagDetection tag24 = getBestTag(RED_TAG);

        if (tag20 == null || tag24 == null || tag20.robotPose == null || tag24.robotPose == null) {
            return null;
        }

        double x = (tag20.robotPose.getPosition().x + tag24.robotPose.getPosition().x) / 2.0;
        double y = (tag20.robotPose.getPosition().y + tag24.robotPose.getPosition().y) / 2.0;
        double heading = tag20.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);

        // BACK-FACING CAMERA
        heading += Math.PI;
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;

        return new Pose(x, y, heading);
    }

    /** Simple velocity table – tune after you record distances */
    private int getVelocity(double distance) {
        if (distance > 35.0) return 1650;
        if (distance > 25.0) return 1500;
        return 1300;
    }

    @Override
    public void stop() {
        if (hardware.visionPortal != null) {
            hardware.visionPortal.close();
        }
    }

    // -----------------------------------------------------------------
    // -------------------------- POSE CLASS ---------------------------
    // -----------------------------------------------------------------
    private static class Pose {
        final double x, y, heading;
        Pose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
        double getX() { return x; }
        double getY() { return y; }
        double getHeading() { return heading; }
    }
}