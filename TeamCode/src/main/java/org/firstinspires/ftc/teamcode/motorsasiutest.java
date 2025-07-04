package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class motorsasiutest extends OpMode {
    DcMotor motorfsstanga, motorfsdreapta, motorspdreapta, motorspstanga;
    IMU imu;
    @Override
     public void init(){
        motorfsstanga = hardwareMap.get(DcMotor.class, "motorfsstanga");
        motorfsdreapta = hardwareMap.get(DcMotor.class, "motorfsdreapta");
        motorspdreapta = hardwareMap.get(DcMotor.class, "motorspdreapta");
        motorspstanga = hardwareMap.get(DcMotor.class, "motorspstanga");

        motorfsdreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfsstanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorspdreapta.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorspstanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
        motorfsdreapta.setDirection(DcMotorSimple.Direction.REVERSE);
        motorspdreapta.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("unghi", botHeading);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorfsdreapta.setPower(frontRightPower);
        motorfsstanga.setPower(frontLeftPower);
        motorspstanga.setPower(backLeftPower);
        motorspdreapta.setPower(backRightPower);

        telemetry.update();

    }
}


