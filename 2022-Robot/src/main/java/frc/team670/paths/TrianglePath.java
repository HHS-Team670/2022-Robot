// package frc.team670.paths.right;

// import java.util.List;

// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import frc.team670.paths.Path;
// import frc.team670.robot.subsystems.DriveBase;
// import frc.team670.robot.constants.RobotConstants;

// /** 
//  * Trajectory starting from the right of the field in line with the trench then through the trench to the last ball (3 balls total)
//  * google doc link: https://docs.google.com/document/d/1AsYxFidJCVz2cBFPJEAD1MU-b7dWLGTkcGLwushU1f8/edit?usp=sharing
//  * @author meganchoy, elisevbp, tarini, rishabh b
//  */
// public class TrianglePath extends Path{
 
//         public TrianglePath(DriveBase driveBase) {
//                 super(

//                         List.of(
//                                 new Pose2d(7.97269028998831, -4.98009098496299, Rotation2d.fromDegrees(68.5522636728949)),
//                                 new Pose2d(5.13715548172964, -6.34309484129333, Rotation2d.fromDegrees(202.166345822082)),
//                                 new Pose2d(7.72929674421503, -7.96166192068562, Rotation2d.fromDegrees(-20.4495476107926)),
//                                 new Pose2d(8.15523544931827, -5.11395743513829, Rotation2d.fromDegrees(77.1957339347133)),
//                         ), 
//                 driveBase, RobotConstants.kAutoPathConstraints, false);
//         }
// }