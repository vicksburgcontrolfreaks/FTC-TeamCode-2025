package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TestTeleOp", group = "TeleOp")
public class TestTeleOp extends OpMode {

    // --------------------------------------------------------------------- //
    // ---------------------------  HARDWARE  ------------------------------ //
    // --------------------------------------------------------------------- //
    private RobotHardware hardware;
    private ShootAprilTag   shooter;      // only for manual Y-align
    private Follower      follower;

    // --------------------------------------------------------------------- //
    // --------------------------  3-SHOT BURST  -------------------------- //
    // --------------------------------------------------------------------- //
    private ThreeShots threeShots;      // <-- NEW reusable burst

    // --------------------------------------------------------------------- //
    // ---------------------------  CONTROLS  ----------------------------- //
    // --------------------------------------------------------------------- //
    private double shooterTPS = 1600.0; // live-adjusted with bumpers
    private boolean collectorOn = false;
    private int tagId = 20;             // changes with alliance

    private final Timer debounceTimer = new Timer();
    private boolean lastA = false, lastB = false;
    private static final double DEBOUNCE_TIME = 0.2;

    // --------------------------------------------------------------------- //
    // ---------------------------  DRIVE  --------------------------------- //
    // --------------------------------------------------------------------- //
    private static final double DEADBAND = 0.1;
    private static final double SLOW_MODE_SPEED = 0.4;
    private boolean headingResetPressed = false;

    // --------------------------------------------------------------------- //
    // --------------------------  ALLIANCE  ------------------------------- //
    // --------------------------------------------------------------------- //
    private boolean isRedAlliance = false;          // false = Blue
    private double fieldForwardHeading = 0.0;        // 0° Red, 180° Blue

    // --------------------------------------------------------------------- //
    // ------------------------------  INIT  ------------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void init() {
        hardware = new RobotHardware();
        hardware.init(hardwareMap, telemetry);

        // ----- follower (field-centric drive) -----
        try {
            follower = Constants.createFollower(hardwareMap);
            hardware.addTelemetry("Status", "Follower initialized");
        } catch (Exception e) {
            hardware.addTelemetry("Error", "Follower failed: " + e.getMessage());
        }

        // ----- alignment helper (Y button) -----
        shooter = new ShootAprilTag(hardware, follower, telemetry);
        shooter.setTelemetryEnabled(false);

        // ----- NEW 3-shot burst -----
        threeShots = new ThreeShots(hardware);
        threeShots.setTelemetryEnabled(true);   // change to false to silence

        // ----- misc -----
        hardware.flipper.setPosition(0.0);
        debounceTimer.resetTimer();

        // default alliance
        setAlliance(false);
    }

    // --------------------------------------------------------------------- //
    // ---------------------------  INIT LOOP  ----------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void init_loop() {
        if (gamepad2.dpad_up)   setAlliance(false); // Blue
        if (gamepad2.dpad_down) setAlliance(true);  // Red

        telemetry.addData("Instructions",
                "Gamepad2: DPAD Up (Blue) / Down (Red)");
        telemetry.addData("Gamepad1",
                "Left Stick (Move), Right Stick (Rotate), Y (Align)");
        telemetry.addData("Gamepad2",
                "A (Collector), Bumpers (TPS), B (3-Shot Burst), X (Flipper)");
        telemetry.addData("Drive",
                "Left Bumper = Slow | Right Bumper = Reset Heading");
        telemetry.addData("Alliance", isRedAlliance ? "RED (0°)" : "BLUE (180°)");
        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Burst Ready", threeShots.isBusy() ? "BUSY" : "READY");
        shooter.updateTelemetry(tagId);
        telemetry.update();
    }

    // --------------------------------------------------------------------- //
    // --------------------------  ALLIANCE SET  -------------------------- //
    // --------------------------------------------------------------------- //
    private void setAlliance(boolean red) {
        isRedAlliance = red;
        fieldForwardHeading = red ? 0.0 : Math.PI;   // 0° or 180°
        tagId = red ? 24 : 20;

        RevBlinkinLedDriver.BlinkinPattern pattern = red ?
                RevBlinkinLedDriver.BlinkinPattern.RED :
                RevBlinkinLedDriver.BlinkinPattern.BLUE;
        hardware.leds.setPattern(pattern);
    }

    // --------------------------------------------------------------------- //
    // ------------------------------  LOOP  ------------------------------- //
    // --------------------------------------------------------------------- //
    @Override
    public void loop() {

        // --------------------------------------------------------------- //
        // -------------------  MANUAL ALIGN (Y)  ----------------------- //
        // --------------------------------------------------------------- //
        if (gamepad1.y) {
            shooter.alignRotationOnly(tagId);
            stopDriveMotors();               // no drive while aligning
            return;                          // skip the rest of the loop
        }

        // --------------------------------------------------------------- //
        // -------------------  FIELD-CENTRIC DRIVE  -------------------- //
        // --------------------------------------------------------------- //
        double rawY  = -gamepad1.left_stick_y;
        double rawX  =  gamepad1.left_stick_x;
        double rawRx =  gamepad1.right_stick_x;

        double y  = Math.abs(rawY)  > DEADBAND ? rawY  : 0.0;
        double x  = Math.abs(rawX)  > DEADBAND ? rawX  : 0.0;
        double rx = Math.abs(rawRx) > DEADBAND ? rawRx : 0.0;

        boolean slowMode = gamepad1.left_bumper;
        double speedMul = slowMode ? SLOW_MODE_SPEED : 1.0;

        // heading reset
        if (gamepad1.right_bumper && !headingResetPressed) {
            follower.getPose().setHeading(fieldForwardHeading);
            telemetry.addData("HEADING", "RESET TO %.0f°",
                    Math.toDegrees(fieldForwardHeading));
        }
        headingResetPressed = gamepad1.right_bumper;

        // field-centric math
        double botHeading = follower.getPose().getHeading() - fieldForwardHeading;
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double max = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        double scale = 1.0 / max;

        double lf = scale * (rotY + rotX + rx) * speedMul;
        double rf = scale * (rotY - rotX - rx) * speedMul;
        double lr = scale * (rotY - rotX + rx) * speedMul;
        double rr = scale * (rotY + rotX - rx) * speedMul;

        hardware.lf.setPower(lf);
        hardware.rf.setPower(rf);
        hardware.lr.setPower(lr);
        hardware.rr.setPower(rr);

        // LED drive-mode visual
        hardware.leds.setPattern(slowMode ?
                RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE :
                (isRedAlliance ?
                        RevBlinkinLedDriver.BlinkinPattern.RED :
                        RevBlinkinLedDriver.BlinkinPattern.BLUE));

        // --------------------------------------------------------------- //
        // -----------------------  LOCALIZER  -------------------------- //
        // --------------------------------------------------------------- //
        follower.update();

        // --------------------------------------------------------------- //
        // --------------------------  COLLECTOR  ---------------------- //
        // --------------------------------------------------------------- //

        if (gamepad2.a) {
            // A = intake forward (collect) @ 0.7 power
            hardware.collector.setPower(0.7);
        } else if (gamepad2.y) {
            // Y = eject (spit out) @ -1.0 power
            hardware.collector.setPower(-1.0);
        } else {
            // neither button → stop (only if burst is idle)
            if (!threeShots.isBusy()) {
                hardware.collector.setPower(0.0);
            }
        }

        // --------------------------------------------------------------- //
        // -----------------------  3-SHOT BURST  ---------------------- //
        // --------------------------------------------------------------- //
        if (gamepad2.b && !lastB) {
            // start a fresh burst at the *current* TPS
            threeShots.start((int) shooterTPS);
        }
        lastB = gamepad2.b;

        // always update the burst (handles spin-up, flip, index, align)
        threeShots.update(tagId);

        // --------------------------------------------------------------- //
        // -----------------------  MANUAL FLIPPER  -------------------- //
        // --------------------------------------------------------------- //
        if (gamepad2.x && !threeShots.isBusy()) {
            hardware.flipper.setPosition(0.5);
        } else if (!threeShots.isBusy()) {
            hardware.flipper.setPosition(0.0);
        }

        // --------------------------------------------------------------- //
        // ---------------------------  TELEMETRY  --------------------- //
        // --------------------------------------------------------------- //
        telemetry.addData("Alliance", isRedAlliance ? "RED" : "BLUE");
        telemetry.addData("Field Forward", "%.0f°", Math.toDegrees(fieldForwardHeading));
        telemetry.addData("Tag ID", tagId);
        telemetry.addData("Shooter TPS Set", shooterTPS);
        telemetry.addData("Shooter Vel", "%.0f", hardware.shooter.getVelocity());
        telemetry.addData("Battery", "%.2fV", hardware.getBatteryVoltage());
        telemetry.addData("Burst", threeShots.isBusy() ? "BUSY" : "READY");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Drive Mode", slowMode ? "SLOW (LB)" : "NORMAL");
        telemetry.update();
    }

    // ----------------------------------------------------------------- //
    // --------------------------  HELPERS  ---------------------------- //
    // ----------------------------------------------------------------- //
    private void stopDriveMotors() {
        hardware.lf.setPower(0);
        hardware.rf.setPower(0);
        hardware.lr.setPower(0);
        hardware.rr.setPower(0);
    }
}