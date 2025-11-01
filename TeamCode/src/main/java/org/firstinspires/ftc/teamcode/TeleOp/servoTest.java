package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.stuff.Robot.reset;
import static org.firstinspires.ftc.teamcode.stuff.Robot.runtime;
import static org.firstinspires.ftc.teamcode.stuff.Robot.servos;
import static org.firstinspires.ftc.teamcode.stuff.Robot.spind;

import org.firstinspires.ftc.teamcode.mechanics.shooterCalc;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.stuff.Robot;
import org.firstinspires.ftc.teamcode.tuning.Constants;

import java.util.ArrayList;

@Config
@TeleOp(name = "Shooter Test", group = "tuning")

public class servoTest extends LinearOpMode {

    public static double dist = 0;
    public int x = 1;
    public static double multiplier = 1;
    public boolean coooooking = true;
    public boolean julian = true;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup everything


        Robot.setup(
                hardwareMap.get(DcMotorEx.class, "front_left_motor"),
                hardwareMap.get(DcMotorEx.class, "back_right_motor"),
                hardwareMap.get(DcMotorEx.class, "front_right_motor"),
                hardwareMap.get(DcMotorEx.class, "back_left_motor"),
                hardwareMap.get(DcMotorEx.class, "spind"),
                hardwareMap.get(DcMotorEx.class, "inta"),
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
//            servos.setHood(shooterCalc.distToAngle(dist));
            servos.setHood(dist);
            Robot.flywheel.setPower(-1);
            if (gamepad1.a) {
                servos.transP1();
            }
            else if (gamepad1.b) {

                servos.transP2();
            }

            else if (gamepad1.y && coooooking) {
                int target = (int)Math.round((Constants.CPR * 93.0 / 22.0 /6.0) * x * multiplier);
                spind.setTargetPosition(target);
                spind.setPower(1);
                spind.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                x += 1;
                coooooking = false;
            }
            else if(!gamepad1.y){
                coooooking = true;
            }
            else if (gamepad1.x && julian) {
                int target = (int)Math.round((Constants.CPR * 93.0 / 22.0 /6.0) * x * multiplier);
                spind.setTargetPosition(target);
                spind.setPower(1);
                spind.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                x -= 1;
                julian = false;
            }
            else if(!gamepad1.x){
                julian = true;
            }

            Robot.inta.setPower(gamepad1.right_stick_x);
            debug();
        }
    }
    public void debug () {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("distance: ", dist );
        dashboardTelemetry.addData("spind pos: ", spind.getCurrentPosition());
        dashboardTelemetry.addData("spind target: ", spind.getTargetPosition());

        dashboardTelemetry.update();

    }
}

