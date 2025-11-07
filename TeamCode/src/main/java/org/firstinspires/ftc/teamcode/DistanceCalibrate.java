package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="DIST CAL", group="Tests")
public class DistanceCalibrate extends OpMode {
    RobotHardware hardware;
    int tagId = 20;

    @Override public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);
        telemetry.addData("CAL", "Hold at 12in, 24in, 36in");
    }

    @Override public void loop() {
        for (AprilTagDetection d : hardware.aprilTagProcessor.getDetections()) {
            if (d.id == tagId && d.ftcPose != null) {
                double dist = d.ftcPose.z;  // forward distance in inches
                telemetry.addData("Reported", "%.1f in", dist);
                break;
            }
        }
        telemetry.update();
    }
}