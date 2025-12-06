package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class LiftSystem {

    private DcMotor lWinch;
    private DcMotor rWinch;
    private Telemetry telemetry;

    private double power = 0.85;

    // Current spike detection parameters
    private static final double CURRENT_THRESHOLD = 6.6; // Amps
    private static final double STARTUP_DELAY = 2.0; // Ignore current spikes for first 2 seconds
    private static final double JOYSTICK_DEADBAND = 0.1; // Ignore small joystick movements
    private boolean currentSpikeDetected = false;
    private boolean autoLiftActive = false;
    private boolean limitReached = false;
    private ElapsedTime startupTimer = new ElapsedTime();

    public LiftSystem(DcMotor lWinch, DcMotor rWinch, Telemetry telemetry) {
        this.lWinch = lWinch;
        this.rWinch = rWinch;
        this.telemetry = telemetry;

        // Set lift motors to run without encoder for manual control
        lWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rWinch.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(Gamepad gamepad2) {
        // Check for current spike on both motors
        double leftCurrent = ((DcMotorEx) lWinch).getCurrent(CurrentUnit.AMPS);
        double rightCurrent = ((DcMotorEx) rWinch).getCurrent(CurrentUnit.AMPS);

        // Detect current spike (but ignore during startup delay)
        boolean inStartupPeriod = startupTimer.seconds() < STARTUP_DELAY;

        if (!inStartupPeriod && (leftCurrent > CURRENT_THRESHOLD || rightCurrent > CURRENT_THRESHOLD)) {
            currentSpikeDetected = true;
            lWinch.setPower(0.0);
            rWinch.setPower(0.0);

            // If auto lift was active, mark limit as reached
            if (autoLiftActive) {
                limitReached = true;
                autoLiftActive = false;
            }
        } else {
            currentSpikeDetected = false;
        }

        // Get left stick Y value (inverted - up is negative)
        double leftStickY = -gamepad2.left_stick_y;

        // Apply deadband
        if (Math.abs(leftStickY) < JOYSTICK_DEADBAND) {
            leftStickY = 0.0;
        }

        // Left stick down (negative after inversion) = Manual lowering (cancels auto lift and clears limit flag)
        if (leftStickY < -JOYSTICK_DEADBAND) {
            autoLiftActive = false;
            limitReached = false;
            // Use joystick value for variable speed control
            lWinch.setPower(leftStickY * power);
            rWinch.setPower(leftStickY * power);
        }
        // Auto lift mode - runs continuously until limit is hit
        else if (autoLiftActive && !currentSpikeDetected) {
            lWinch.setPower(power);
            rWinch.setPower(power);
        }
        // Left stick up (positive) = Start auto lift to limit (only if not already active)
        else if (leftStickY > JOYSTICK_DEADBAND && !autoLiftActive) {
            autoLiftActive = true;
            limitReached = false;  // Reset limit flag when starting new auto lift
            startupTimer.reset();  // Reset timer to ignore current spikes during startup
            lWinch.setPower(power);
            rWinch.setPower(power);
        }
        // No input and not auto lifting = Stop
        else if (!autoLiftActive) {
            lWinch.setPower(0.0);
            rWinch.setPower(0.0);
        }
    }

    public void updateTelemetry() {
        double leftCurrent = ((DcMotorEx) lWinch).getCurrent(CurrentUnit.AMPS);
        double rightCurrent = ((DcMotorEx) rWinch).getCurrent(CurrentUnit.AMPS);

        telemetry.addData("Left Winch Encoder", lWinch.getCurrentPosition());
        telemetry.addData("Right Winch Encoder", rWinch.getCurrentPosition());
        telemetry.addData("Left Winch Power", lWinch.getPower());
        telemetry.addData("Right Winch Power", rWinch.getPower());
        telemetry.addData("Left Current (A)", "%.2f", leftCurrent);
        telemetry.addData("Right Current (A)", "%.2f", rightCurrent);
        telemetry.addData("Auto Lift", autoLiftActive ? "ACTIVE" : "Inactive");
        telemetry.addData("Startup Period", startupTimer.seconds() < STARTUP_DELAY ? "YES (ignoring spikes)" : "No");
        telemetry.addData("Limit Reached", limitReached ? "YES - LEDS FLASHING" : "No");
        telemetry.addData("Current Spike", currentSpikeDetected ? "DETECTED - STOPPED!" : "Normal");
        telemetry.addData("Current Threshold (A)", CURRENT_THRESHOLD);
    }

    public void setPower(double power) {
        this.power = power;
    }

    public double getPower() {
        return power;
    }

    public boolean isCurrentSpikeDetected() {
        return currentSpikeDetected;
    }

    public boolean isLimitReached() {
        return limitReached;
    }

    public boolean isAutoLiftActive() {
        return autoLiftActive;
    }

    public void stop() {
        lWinch.setPower(0.0);
        rWinch.setPower(0.0);
    }
}
