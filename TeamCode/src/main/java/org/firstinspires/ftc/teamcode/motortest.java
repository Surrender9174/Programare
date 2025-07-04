package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class motortest extends OpMode {

    DcMotor motor;
    Servo servo;

    double pos = 0.d;
    private boolean sign = false;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(1);

        if ( gamepad1.left_trigger > 0.2)
            pos -= 0.002 * gamepad1.left_trigger;

        if ( gamepad1.right_trigger > 0.2)
            pos += 0.002 * gamepad1.right_trigger;

        servo.setPosition(pos);
    }
}
