package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "TEST: AprilTag Center", group = "Tests")
public class AprilTagCenterTest extends OpMode {

    private RobotHardware hardware;

    // Change this to the tag you are using for the test
    private static final int TEST_TAG_ID = 20;          // 20 = Blue, 24 = Red
    private static final double TARGET_CENTER_X = 640.0; // 1280 px width → center
    private static final double TOLERANCE_PX    = 23.0;

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        telemetry.addData("STATUS", "AprilTag Center Test Ready");
        telemetry.addData("INSTRUCTIONS", "Rotate robot by hand");
        telemetry.addData("GOAL", "center.x ≈ %.0f (±%.0f px)", TARGET_CENTER_X, TOLERANCE_PX);
        telemetry.addData("Camera", "Anker C200 – 78° FOV – 1280×720");
        telemetry.update();
    }

    @Override
    public void loop() {
        List<AprilTagDetection> detections = hardware.aprilTagProcessor.getDetections();
        AprilTagDetection target = null;

        // Find the tag we care about
        for (AprilTagDetection d : detections) {
            if (d.id == TEST_TAG_ID) {
                target = d;
                break;
            }
        }

        if (target != null) {
            double centerX = target.center.x;
            double error   = TARGET_CENTER_X - centerX;

            telemetry.addData("TAG ID", target.id);
            telemetry.addData("center.x", "%.1f", centerX);
            telemetry.addData("Error (±640)", "%.1f", error);
            telemetry.addData("IN RANGE?", Math.abs(error) <= TOLERANCE_PX ? "YES" : "NO");

            if (Math.abs(error) > TOLERANCE_PX) {
                telemetry.addData("Rotate Robot", error > 0 ? "LEFT" : "RIGHT");
            } else {
                telemetry.addData("Rotate Robot", "GOOD – stop!");
            }
        } else {
            telemetry.addData("TAG ID", "NOT FOUND");
            telemetry.addData("center.x", "—");
            telemetry.addData("Error", "—");
        }

        telemetry.addData("Detections", detections.size());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Graceful shutdown – stops the camera stream
        if (hardware.visionPortal != null) {
            hardware.visionPortal.close();
        }
    }
}