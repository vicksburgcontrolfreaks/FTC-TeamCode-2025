package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "INDEX TEST", group = "Tests")
public class IndexTest extends OpMode {

    private DcMotorEx collector;

    @Override
    public void init() {
        collector = hardwareMap.get(DcMotorEx.class, "collector");
        collector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("INDEX TEST", "Press Play → A");
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            int targetCollectorPos = collector.getCurrentPosition() + 500;
            collector.setTargetPosition(targetCollectorPos);
            collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            collector.setPower(0.9);
            //Scenario A: Math in If statement determines when done
            if (Math.abs(collector.getCurrentPosition() - targetCollectorPos) < 30){
                collector.setPower(0);
            } else {
                //just wait
            }

            telemetry.addData("GO", "Indexing to " + targetCollectorPos);
            telemetry.addData("Pos", collector.getCurrentPosition());
            telemetry.addData("Busy", collector.isBusy() ? "YES" : "DONE");
        }

        if (gamepad1.b) {
            int targetCollectorPos = collector.getCurrentPosition() + 500;
            collector.setTargetPosition(targetCollectorPos);
            collector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            collector.setPower(0.9);
            //Scenario B: motor position tolerance determines when done
            collector.setTargetPositionTolerance(30);
            if (!collector.isBusy()){
                collector.setPower(0);
                collector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            } else {
                //just wait
            }

            telemetry.addData("GO", "Indexing to " + targetCollectorPos);
            telemetry.addData("Pos", collector.getCurrentPosition());
            telemetry.addData("Busy", collector.isBusy() ? "YES" : "DONE");
        }
    }
}