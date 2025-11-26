package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.FIELD_CENTRIC;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.absolute_drive;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.accelerate;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.field_centric_drive;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.robot_centric_drive;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.rotational_power;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.x_power;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.y_power;
import static org.firstinspires.ftc.teamcode.navigation.Motion.go_to;
import static org.firstinspires.ftc.teamcode.stuff.Robot.X;
import static org.firstinspires.ftc.teamcode.stuff.Robot.Y;
import static org.firstinspires.ftc.teamcode.stuff.Robot.begin;
import static org.firstinspires.ftc.teamcode.stuff.Robot.end;
import static org.firstinspires.ftc.teamcode.stuff.Robot.flywheel;
import static org.firstinspires.ftc.teamcode.stuff.Robot.imu;
import static org.firstinspires.ftc.teamcode.stuff.Robot.initial_theta;
import static org.firstinspires.ftc.teamcode.stuff.Robot.inta;
import static org.firstinspires.ftc.teamcode.stuff.Robot.reset;
import static org.firstinspires.ftc.teamcode.stuff.Robot.runtime;
import static org.firstinspires.ftc.teamcode.stuff.Robot.servos;
import static org.firstinspires.ftc.teamcode.stuff.Robot.settings;
import static org.firstinspires.ftc.teamcode.stuff.Robot.spind;
import static org.firstinspires.ftc.teamcode.stuff.Robot.theta;
import static org.firstinspires.ftc.teamcode.stuff.Robot.turn;
import static org.firstinspires.ftc.teamcode.tuning.Constants.LENGTH;
import static org.firstinspires.ftc.teamcode.tuning.Constants.TELEOP_FIELD;
import static org.firstinspires.ftc.teamcode.tuning.Constants.TILE_LENGTH;
import static org.firstinspires.ftc.teamcode.tuning.Recipes.desired_linear_slide_position;
import static org.firstinspires.ftc.teamcode.tuning.Recipes.desired_theta;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mathemagics.Point;
import org.firstinspires.ftc.teamcode.mechanics.Drivetrain;
import org.firstinspires.ftc.teamcode.mechanics.ServoErmSigma;
import org.firstinspires.ftc.teamcode.navigation.Motion;
import org.firstinspires.ftc.teamcode.stuff.Robot;
import org.firstinspires.ftc.teamcode.tuning.Constants;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "🥶 TeleOp", group = "Comp Run")

public class BlueT extends LinearOpMode {

    public boolean b1 = false;
    public boolean y1 = false;
    public boolean x1 = false;
    public boolean a1 = false;

    public boolean up1 = false;
    public boolean down1 = false;

    public boolean a2 = false;
    public boolean b2 = false;
    public boolean y2 = false;
    public boolean x2 = false;
    public boolean right_bumper2 = false;
    public boolean left_bumper2 = false;
    public boolean up_bumper2 = false;
    public boolean down_bumper2 = false;

    public boolean shooterOn = true;
    public boolean joyBoy = false;
    public boolean coooooking = true;
    public boolean coooooking2 = true;
    public int x = 1;


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
                hardwareMap.get(DcMotorEx.class, "turn"),
                hardwareMap.get(DcMotorEx.class, "shot"),
                hardwareMap.get(IMU.class, "imu"),
                new ArrayList<>(hardwareMap.getAll(LynxModule.class)),
                true
        );

        // initial position
        X = 0 * TILE_LENGTH;
        Y = 0 * TILE_LENGTH;

        // initial theta
        initial_theta = Math.PI;
        theta = Math.PI;
        desired_theta = Math.PI;

        // scoring
        desired_linear_slide_position = 0;

        Queue<Point> locations = new LinkedList<>();

        // waiting...
        waitForStart();
        runtime.reset();

        //when have auto put this in that file and remove from here
        reset(hardwareMap);

        // key presses
        b1 = false;
        y1 = false;
        x1 = false;
        a1 = false;

        up1 = false;
        down1 = false;


        // driver 1
        new Thread(() -> {
            while (opModeIsActive()) {
                settings(gamepad1.left_stick_button, gamepad1.right_stick_button);

                if (gamepad1.b && !b1) {
                    desired_theta = theta - Math.toRadians(45);

                    b1 = true;
                } else if (!gamepad1.b) {
                    b1 = false;
                }
                //added this for sigma driving
                if(gamepad1.start){
                    imu.resetYaw();
                }
                //take this out fo rthe meet, just so we dont destroy our ears
                if(gamepad1.right_stick_button&&!joyBoy) {
                    if(shooterOn) {
                        flywheel.setPower(0);
                    }else{
                        flywheel.setPower(-1);
                    }

                    joyBoy = true;
                }else if(!gamepad1.right_stick_button){
                    joyBoy=false;
                }
                if (gamepad1.y && !y1) {
                    desired_theta = theta + Math.toRadians(45);

                    y1 = true;
                } else if (!gamepad1.y) {
                    y1 = false;
                }

                if (gamepad1.a && !a1) {
                    locations.add(new Point(1.5 * TILE_LENGTH, 0.5 * TILE_LENGTH));

                    a1 = true;
                } else if (!gamepad1.a) {
                    a1 = false;
                }

                if (gamepad1.dpad_up && !up1) {
                    locations.add(new Point(-0.5 * TILE_LENGTH, -1.5 * TILE_LENGTH));
                    locations.add(new Point(-1.5 * TILE_LENGTH, -1.5 * TILE_LENGTH));

                    desired_theta = Math.toRadians(225);

                    up1 = true;
                } else if (!gamepad1.dpad_up) {
                    up1 = false;
                }

                if (gamepad1.dpad_down && !down1) {
                    locations.add(new Point(1.5 * TILE_LENGTH, 1.5 * TILE_LENGTH));

                    down1 = true;
                } else if (!gamepad1.dpad_down) {
                    down1 = false;
                }

                if (gamepad1.x && !x1) {
                    locations.clear();
                    Motion.path = null;

                    x_power = 0;
                    y_power = 0;
                    rotational_power = 0;

                    x1 = true;
                } else if (!gamepad1.x) {
                    x1 = false;
                }

                if (gamepad1.right_stick_x != 0) {
                    desired_theta = theta;
                }
            }
        }).start();

        // driver 2
        new Thread(() -> {
            while (opModeIsActive()) {
                servos.setHood(0.5);
                // first mark
                if (gamepad2.right_trigger > 0.6) {

                    // goofy ahh sleep
                    try { Thread.sleep(300); } catch (Exception ignored) {}
                    //commented because servo was stalling, temporary
                    //servos.transP1();
                }

                if (gamepad2.left_trigger > .6) {
                    //servos.transP2();
                }

                inta.setPower(gamepad2.left_stick_y);
                if (Math.abs(gamepad2.left_stick_y) > .3) {
                    servos.intake();
                }
                else if (gamepad2.y && coooooking) {
                    int target = (int)Math.round((Constants.CPR * 93.0 / 22.0 /6.0) * x);
                    spind.setTargetPosition(target);
                    spind.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spind.setPower(1);
                    x += 1;
                    servos.spin();
                    coooooking = false;
                }
                else if(!gamepad2.y){
                    coooooking = true;
                    servos.stop();
                }
                else if (gamepad2.dpad_left && coooooking2) {
                    int target = -(int)Math.round((Constants.CPR * 93.0 / 22.0 /6.0) * x);
                    spind.setTargetPosition(target);
                    spind.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    spind.setPower(1);
                    x -= 1;
                    servos.spin();
                    coooooking2 = false;
                }
                else if(!gamepad2.dpad_left){
                    coooooking2 = true;
                    servos.stop();
                }
                //flywheel.setPower(-1);
                turn.setPower(gamepad2.right_stick_x);


                servos.intake();
            }
        }).start();

        while (opModeIsActive()) {
            begin();

            // angular
            rotational_power = Drivetrain.rotational_power + gamepad1.right_stick_x;
           // turn_to(desired_theta, gamepad1.right_trigger);

            if (locations.isEmpty()) {
                // translational
                accelerate(gamepad1.left_stick_y, gamepad1.left_stick_x);

                if (FIELD_CENTRIC) {
                    field_centric_drive(
                            y_power * (1 - 0.75 * gamepad1.left_trigger),
                            x_power * (1 - 0.75 * gamepad1.left_trigger),
                            rotational_power * (1 - 0.75 * gamepad1.left_trigger)
                    );
                } else {
                    robot_centric_drive(
                            y_power * (1 - 0.75 * gamepad1.left_trigger),
                            x_power * (1 - 0.75 * gamepad1.left_trigger),
                            rotational_power * (1 - 0.75 * gamepad1.left_trigger)
                    );
                }
            } else {
                if (go_to(locations.peek(), TELEOP_FIELD)){
                    locations.poll();

                    Motion.path = null;
                }

                absolute_drive(y_power, x_power, rotational_power);
            }


            //claw.rotToDeg((angle_to_horizontal + desired_claw_angle + 10) * Math.signum(Math.cos(Arm.POSITION)));


            end();

            debug();
        }
    }

    public void debug() {
        telemetry.addData("IMU: ", Robot.imu.getRobotYawPitchRollAngles());

        telemetry.addData("X: ", Robot.X);
        telemetry.addData("Y: ", Robot.Y);

        telemetry.addData("theta: ", Robot.theta);

        telemetry.addData("field centric?", Drivetrain.FIELD_CENTRIC);

        telemetry.addData("imu data", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        telemetry.addData("diffyL", ServoErmSigma.hood.getPosition());

        telemetry.update();
    }
}