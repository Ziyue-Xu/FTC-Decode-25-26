package org.firstinspires.ftc.teamcode.navigation;

import org.firstinspires.ftc.teamcode.mathemagics.Point;
import org.firstinspires.ftc.teamcode.tuning.Constants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class LocalizationV2_AprilTagLoc {

    public Point getRobotPos(AprilTagDetection tag) {
        double distToPoint = tag.ftcPose.range;
        double bearing = tag.ftcPose.bearing;
        double yaw = tag.ftcPose.yaw;
        double x = distToPoint*Math.sin(yaw-bearing);
        double y = distToPoint*Math.cos(yaw-bearing);
        Point tagPos = Constants.aprilTag1Pos;
        return new Point(tagPos, x, y);
    }

    public double getRobotHeading(AprilTagDetection tag) {
        double yaw = 90+tag.ftcPose.yaw;
        int x = 1;
        if (yaw > 0)
            x = -1;
        return Constants.aprilTag1Heading+90*x+yaw;
    }
}
