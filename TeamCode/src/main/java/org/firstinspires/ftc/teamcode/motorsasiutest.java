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
    DcMotor motorfst, motorfdr, motorsdr, motorsst, motorscr;
    IMU imu;
    @Override
     public void init(){
        motorfst = hardwareMap.get(DcMotor.class, "mfs");
        motorfdr = hardwareMap.get(DcMotor.class, "mfd");
        motorsst = hardwareMap.get(DcMotor.class, "mss");
        motorsdr = hardwareMap.get(DcMotor.class, "msd");
        motorscr = hardwareMap.get(DcMotor.class, "motorscripete");

        motorfdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsdr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorsst.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorscr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorscr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorfdr.setDirection(DcMotorSimple.Direction.REVERSE);
        motorsdr.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        double scripete = gamepad1.left_trigger - gamepad1.right_trigger;
        motorfdr.setPower(frontRightPower);
        motorfst.setPower(frontLeftPower);
        motorsst.setPower(backLeftPower);
        motorsdr.setPower(backRightPower);
        motorscr.setPower(scripete);
        if(gamepad1.options)
            imu.resetYaw();
        telemetry.addData("Pozitie", motorscr.getCurrentPosition());
        telemetry.update();
        if(gamepad1.left_bumper){
            double starttime = getRuntime();
            double endtime = starttime;
            while(endtime - starttime > 2000000
            ){
                endtime = getRuntime();
                motorscr.setPower(1);
            }
            motorscr.setPower(0.5);
        }
    }
}

