package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TEST: Align Only", group = "Tests")
public class AlignOnlyTest extends OpMode {

    private RobotHardware hardware;
    private AlignAprilTag aligner;

    private static final int TAG_ID = 20;  // Change to 24 for red

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        aligner = new AlignAprilTag(hardware, null, telemetry);
        aligner.setTelemetryEnabled(true);

        telemetry.addData("ALIGN TEST", "Press A to align");
        telemetry.addData("Tag ID", TAG_ID);
        telemetry.addData("Target X", "%.0f ± 23", 640.0);
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            aligner.alignRotationOnly(TAG_ID);
        } else {
            // Stop motors when A is released
            hardware.lf.setPower(0);
            hardware.rf.setPower(0);
            hardware.lr.setPower(0);
            hardware.rr.setPower(0);
        }

        // Always show tag status
        aligner.updateTelemetry(TAG_ID);
    }

    @Override
    public void stop() {
        if (hardware.visionPortal != null) {
            hardware.visionPortal.close();
        }
    }
}