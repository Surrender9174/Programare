package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class motortest extends OpMode {

    DcMotor motorss, motorfs;
    @Override
    public void init() {
        motorss = hardwareMap.get(DcMotor.class, "msd");
        motorfs = hardwareMap.get(DcMotor.class, "mfd");
    }
    @Override
    public void loop() {
        motorss.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorss.setPower(gamepad1.left_stick_x);
        motorfs.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfs.setPower(gamepad1.left_stick_y);

    }
}
