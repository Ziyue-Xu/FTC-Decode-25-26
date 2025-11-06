package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.stuff.Robot;

import java.util.ArrayList;
@Autonomous(name = "drive", group = "Comp run")
public class BlueTeamAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        waitForStart();
        Robot.setup(
                hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                hardwareMap.get(DcMotorEx.class, "back_right_motor"),
                hardwareMap.get(DcMotorEx.class, "front_right_motor"),
                hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                hardwareMap.get(DcMotorEx.class, "spind"),
                hardwareMap.get(DcMotorEx.class, "inta"),
                hardwareMap.get(DcMotorEx.class, "turn"),
                hardwareMap.get(DcMotorEx.class, "shot"),
                hardwareMap.get(IMU.class, "imu"),
                new ArrayList<>(hardwareMap.getAll(LynxModule.class)),
                true
        );
        while (opModeIsActive()){

            Robot.front_right_motor.setPower(1);

            Robot.front_left_motor.setPower(-1);
            Robot.back_left_motor.setPower(1);
            Robot.back_right_motor.setPower(-1);
        }
    }
}
