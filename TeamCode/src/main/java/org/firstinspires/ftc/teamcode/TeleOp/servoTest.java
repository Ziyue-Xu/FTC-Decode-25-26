package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.stuff.Robot.reset;
import static org.firstinspires.ftc.teamcode.stuff.Robot.runtime;
import static org.firstinspires.ftc.teamcode.stuff.Robot.servos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.stuff.Robot;

import java.util.ArrayList;

@Config
@TeleOp(name = "Servo Test", group = "Linear OpMode")

public class servoTest extends LinearOpMode {

    public static double hoodPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup everything


        Robot.setup(
                hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                hardwareMap.get(DcMotorEx.class, "back_right_motor"),
                hardwareMap.get(DcMotorEx.class, "front_right_motor"),
                hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                hardwareMap.get(DcMotorEx.class, "spind"),
                hardwareMap.get(DcMotorEx.class, "int"),
                hardwareMap.get(DcMotorEx.class, "turret"),
                hardwareMap.get(DcMotorEx.class, "shot"),
                hardwareMap.get(IMU.class, "imu"),
                new ArrayList<>(hardwareMap.getAll(LynxModule.class)),
                true
        );


        // waiting...
        waitForStart();
        runtime.reset();
        reset(hardwareMap);

        while (opModeIsActive()) {
            servos.setHood(hoodPos);
            Robot.flywheel.setPower(1);
            if (gamepad1.a) {
                servos.transP1();
                servos.transP2();
            }
        }
    }
    public void debug () {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("hood pos: ", hoodPos );

        dashboardTelemetry.update();

    }
}

