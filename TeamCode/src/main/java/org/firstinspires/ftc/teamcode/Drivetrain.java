package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.Arrays;
import java.util.List;

public class Drivetrain {
    private final DcMotorEx frontLeft, frontRight, rearLeft, rearRight;
    private List<DcMotorEx> motors;

    public Drivetrain(HardwareMap hm) {
        this.frontLeft = hm.get(DcMotorEx.class, "frontLeft");
        this.frontRight = hm.get(DcMotorEx.class, "frontRight");
        this.rearLeft = hm.get(DcMotorEx.class, "rearLeft");
        this.rearRight = hm.get(DcMotorEx.class, "rearRight");

        motors = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);
    }

    public void update(double forward, double strafe, double turn, double heading,
                       double slow_input) {
        double forward_rot = forward * Math.cos(heading) + strafe * Math.sin(heading);
        double strafe_rot = -forward * Math.sin(heading) + strafe * Math.cos(heading);

        frontLeft.setPower(Range.scale(forward_rot + strafe_rot + turn, 0, 1, 0, 1));
        frontRight.setPower(Range.scale(forward_rot - strafe_rot - turn, 0, 1, 0, 1));
        rearLeft.setPower(Range.scale(forward_rot - strafe_rot + turn, 0, 1, 0, 1));
        rearRight.setPower(Range.scale(forward_rot + strafe_rot - turn, 0, 1, 0, 1));
    }

    public void setMotorsDirection(
            boolean frontLeftInv, boolean frontRightInv,
            boolean rearLeftInv, boolean rearRightInv
    )
    {
        frontLeft.setDirection(frontLeftInv ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(frontRightInv ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(rearLeftInv ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rearRight.setDirection(rearRightInv ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }
    public void slowDown(double slow_input) {
        slow_input = Math.max(0, Math.min (slow_input, 1));
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }
}
