package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class TestOP  extends OpMode {
    private Drivetrain drivetrain;
    @Override
    public void init() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double heading = 0; // Replace with actual heading from IMU
        double slow_input = Math.max(0.1, 0.5 - gamepad1.left_trigger );

        drivetrain.update(forward, strafe, turn, heading, slow_input);

        telemetry.addData("Driving Mode", "TeleOp");

        telemetry.addData("frontLeft", drivetrain.getMotors().get(0).getPower());
        telemetry.addData("frontRight", drivetrain.getMotors().get(1).getPower());
        telemetry.addData("rearLeft", drivetrain.getMotors().get(2).getPower());
        telemetry.addData("rearRight", drivetrain.getMotors().get(3).getPower());
        telemetry.addData("Slow mode", slow_input);
        telemetry.update();
    }
}



