package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class RobotPose {
    /// Stores the robot pose from autonomous, which is read in teleop. Change 'updated' to tell teleop not to mess up
    public static Pose2d lastRobotPose = null;
    public static boolean updated = false;
}