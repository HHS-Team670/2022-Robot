package frc.team670.paths;

import java.util.List;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.team670.paths.Path;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;

public class ATarmacEdgeBall extends Path{
    public ATarmacEdgeBall(DriveBase driveBase) {
        super(List.of(
            new Pose2d(7.07575732243747, -3.55751201127157, Rotation2d.fromDegrees(143.343891584033)),
            new Pose2d(4.38135323649172, -1.63909630207819, Rotation2d.fromDegrees(150.478638165418))),
            driveBase, RobotConstants.kAutoPathConstraints, true
        );
    }
}
