package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.teamcode.stuff.*;
import org.firstinspires.ftc.teamcode.mathemagics.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

//camera imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

//mechanics import
import org.firstinspires.ftc.teamcode.mechanics.Spindexer;
import static org.firstinspires.ftc.teamcode.tuning.Constants.*;
import static org.firstinspires.ftc.teamcode.stuff.Robot.*;
import static org.firstinspires.ftc.teamcode.tuning.Recipes.*;
import static org.firstinspires.ftc.teamcode.mechanics.Drivetrain.*;
import static org.firstinspires.ftc.teamcode.navigation.Motion.*;

import com.acmerobotics.dashboard.config.Config;

import java.util.*;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "(red far side) 🥵 Autonomous", group = "Comp run")

public class redFarSide extends LinearOpMode {


    public boolean cooked = false;
    public boolean cooking = false;

    public double begin_wait = -1;
    public double wait = 0.5;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    String motif = "";
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // setup
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
        Robot.X = -0.5 * TILE_LENGTH;
        Robot.Y = -3 * TILE_LENGTH + LENGTH / 2;
        desired_position = new Point(X, Y);

        // initial theta
        //0 is facing right then goes clockwise
        initial_theta = 0;
        theta = 0;
        desired_theta = 0;

        // COOK 🗣‼️
        Queue<Ingredient[]> POT = new LinkedList<>();


        waitForStart();
        runtime.reset();


        reset(hardwareMap);


        Thread.sleep(250);

        begin_wait = -1;
        wait = 0.5;

        cooked = false;
        cooking = false;


        //Camera code
        initAprilTag();
        if (USE_WEBCAM)
            setManualExposure(6, 250);
        waitForStart();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for(AprilTagDetection detection: currentDetections){
            if(motif.isEmpty()) {
                switch (detection.id) {
                    case 21:
                        motif = "GPP";
                        visionPortal.stopStreaming();
                        POT.addAll(Arrays.asList(redFarSideGPP));
                        break;
                    case 22:
                        motif = "PGP";
                        visionPortal.stopStreaming();
                        POT.addAll(Arrays.asList(redFarSidePGP));
                        break;
                    case 23:
                        motif = "PPG";
                        visionPortal.stopStreaming();
                        POT.addAll(Arrays.asList(redFarSidePPG));
                        break;
                    default:
                        telemetry.addData("Skipping", "this sigma is NOT the obelisk", detection.id);
                }

            }
        }
        while (opModeIsActive()) {

            begin();

            if (!POT.isEmpty() && Robot.time - begin_wait > wait) {
                Ingredient[] recipe = POT.peek();

                // always wait 0.1 seconds before each recipe
                wait = 0.1;

                for (Ingredient ingredient : recipe) {
                    switch (ingredient.type) {
                        case "position":
                            desired_position = ingredient.position;
                            break;
                        case "theta":
                            desired_theta = ingredient.value;
                            break;
                        case "wait":
                            begin_wait = Robot.time;
                            wait = ingredient.value;
                            break;
                        case "shooter":
                            flywheel.setPower(-ingredient.value);
                        case "spindexer":
                            Spindexer.advance((int)ingredient.value);
                        case "servo":
                            if (ingredient.servo.equals("transfer")) {
                                servos.transP1();
                                try {
                                    Thread.sleep(50);
                                } catch (Exception ignored) {
                                }
                                servos.transP2();
                            }
                            else if (ingredient.servo.equals("hood")) {
                                servos.setHood(ingredient.distance);
                            }
                            break;
                    }
                }
            }


            cooking = true;


            cooked = go_to(desired_position, TELEOP_FIELD);
            telemetry.addData("desired_position.x", desired_position.x);
            telemetry.addData("desired_position.y", desired_position.y);
            cooked = turn_to(desired_theta, 1) && cooked;
            telemetry.addData("desired_theta", cooked);
            telemetry.addData("desired_theta", desired_theta);

            if (cooking && cooked) {
                // establish wait first
                begin_wait = Robot.time;

                POT.poll();

                cooking = false;

                path = null;
            }

            absolute_drive(y_power, x_power, rotational_power);

            debug();

            end();
        }
    }


    public void debug() {

        telemetry.addData("X: ", Robot.X);
        telemetry.addData("Y: ", Robot.Y);

        telemetry.addData("x power: ", x_power);
        telemetry.addData("y power: ", y_power);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("x power: ", x_power);
        dashboardTelemetry.addData("y power: ", y_power);

        dashboardTelemetry.addData("X: ", X);
        dashboardTelemetry.addData("Y: ", Y);

//        dashboardTelemetry.addData("Motif: ", motif);

        dashboardTelemetry.update();

        telemetry.update();
    }

    //camera code methods
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // e.g. Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}