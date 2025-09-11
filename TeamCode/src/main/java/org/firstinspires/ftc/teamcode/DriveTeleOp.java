package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "DriveTeleOp", group = "TeleOp")
public class DriveTeleOp extends OpMode {
    private RobotHardware hardware;
    private ShootAprilTag shooter;
    private Follower follower;
    private int tagId = 20; // Default to blue alliance (ID 20)

    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap);
        shooter = new ShootAprilTag(hardware);
        try {
            follower = Constants.createFollower(hardwareMap);
            telemetry.addData("Status", "Follower initialized");
        } catch (Exception e) {
            telemetry.addData("Error", "Follower failed: " + e.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        // Alliance selection
        if (gamepad1.dpad_up) {
            tagId = 20; // Blue alliance
            telemetry.addData("Alliance", "Blue (Tag ID 20)");
        } else if (gamepad1.dpad_down) {
            tagId = 24; // Red alliance
            telemetry.addData("Alliance", "Red (Tag ID 24)");
        }

        // Field-centric drive
        double y = -gamepad1.left_stick_y; // Forward/back
        double x = gamepad1.left_stick_x; // Strafe
        double rx = gamepad1.right_stick_x; // Rotate

        // Rotate for field-centric (use Pinpoint heading from Follower)
        double heading = follower.getPose().getHeading();
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        // Denominator for wheel powers
        double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        // Set powers
        hardware.lf.setPower((rotY + rotX + rx) / den);
        hardware.rf.setPower((rotY - rotX - rx) / den);
        hardware.lr.setPower((rotY - rotX + rx) / den);
        hardware.rr.setPower((rotY + rotX - rx) / den);

        // Collect: Gamepad1 A - run collector until full
        if (gamepad1.a) {
            if (!hardware.isMagazineFull()) {
                hardware.collector.setPower(0.5); // Run roller
            } else {
                hardware.collector.setPower(0);
                telemetry.addData("Magazine", "Full");
            }
        } else {
            hardware.collector.setPower(0);
        }

        // Shoot: Gamepad1 B - call shooting sequence with selected tagId
        if (gamepad1.b) {
            shooter.shoot(tagId); // Fixed: Pass tagId
        }

        // Update localizer (via Follower)
        follower.update();

        telemetry.addData("Heading", Math.toDegrees(heading));
        telemetry.addData("Tag ID", tagId);
        telemetry.update();
    }
}