package frc.team670.paths.right;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.team670.paths.Path;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.constants.RobotConstants;

/** 
 * Trajectory starting from the right of the field in line with the trench then through the trench to the last ball (3 balls total)
 * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
 * @author meganchoy, elisevbp, tarini, rishabh b
 */
public class FourBA extends Path{
 
        public FourBA(DriveBase driveBase) {
                super(

                        List.of(
                            new Pose2d(7.63248007592684, -6.83635272130499, Rotation2d.fromDegrees(268.152389734006)),
                            new Pose2d(7.64760496608996, -8.07659371468116, Rotation2d.fromDegrees(-88.9190758133393)),
                            new Pose2d(8.05597700049431, -4.91549167058823, Rotation2d.fromDegrees(258.231711067979)),
                            new Pose2d(4.84950028591201, -6.47335535739001, Rotation2d.fromDegrees(118.610459665965)),
                            new Pose2d(4.92512473672763, -1.87538874780029, Rotation2d.fromDegrees(-39.1736579704445)),
                            new Pose2d(7.54173073494809, -4.06849782145328, Rotation2d.fromDegrees(-27.8014587799342))
                        ), 

                driveBase, RobotConstants.kAutoPathConstraints, false);
        }
}