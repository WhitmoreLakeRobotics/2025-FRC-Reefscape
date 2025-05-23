// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import swervelib.math.Matter;
import edu.wpi.first.math.util.Units;

//
/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be
 * declared globally (i.e. public static). Do not put anything functional in
 * this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
  /**
   * public static final class DriveConstants {
   * public static final int kLeftMotor1Port = 0;
   * public static final int kLeftMotor2Port = 1;
   * public static final int kRightMotor1Port = 2;
   * public static final int kRightMotor2Port = 3;
   * }
   */

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(15.39); // need to update

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.01;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class CanIds {
    // Can id 1-8 are for the drive train
    public static final int ELEVATOR_MOTOR = 9;
    public static final int ARM_MOTOR = 10;
    public static final int CORAL_MOTOR = 11;
    public static final int ALGAE_PIVOT_MOTOR = 12;
    public static final int ALGAE_INTAKE_MOTOR = 13;
    public static final int BOTTOM_LIFT_MOTOR = 14;
    public static final int RIGHT_GUIDE = 18;
    public static final int LEFT_GUIDE = 16;
    public static final int CLIMB_MOTOR = 17;
    public static final int FUNNEL_MOTOR = 15;
  }

   public static final class VisionConstants{
    public static final  double maximumAmbiguity = 0.25;
    public static final double debouncetimeMilli = 15;

   /*  public static final class CameraTemplate{
      public static final String name = "tempalte"; // tempalte?
      public static  Rotation3d robotToCamTransform = new Rotation3d(0, Units.degreesToRadians(18), 0);
      public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(-4.628),
      Units.inchesToMeters(-10.687),
      Units.inchesToMeters(16.129));
      public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(2, 2, 8);
      public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
    }

    public static final class RIGHT_CAM{
      public static final String name = "right";
      public static  Rotation3d robotToCamTransform = new Rotation3d(0, 0, Units.degreesToRadians(-45));
      public static  Translation3d robotToCamTranslation=new Translation3d(Units.inchesToMeters(0),
      Units.inchesToMeters(0),
      Units.inchesToMeters(0));
      public static  Matrix<N3, N1> singleTagStdDevs= VecBuilder.fill(4, 4, 8);
      public static  Matrix<N3, N1> multiTagStdDevsMatrix= VecBuilder.fill(0.5, 0.5, 1); 
      public static InterpolatingDoubleTreeMap stdDevsMap = new InterpolatingDoubleTreeMap();
    }
  */
    

    public static final double leftAlignmentX = .2435737274077523; //meters
    public static final double leftAlignmentY = 0.26;
    public static final double rightAlignmentX = .2435737274077523;
    public static final double rightAlignmentY = 0.6354460866293814;
    public static final double thetaAlignment = -Math.PI/2; //degrees
    public static double maxAlignmentDistance = 1.5;

    public static final double xTolerance = .05;
    public static final double yTolerance = .05;
    public static final double thetaTolerance = .05;
  }

  public static class SwerveConstants {
    public static final double kPX = 1.5;
    public static final double kPY = 1.25;
    public static final double kPTheta = 0.08;
    public static final double kXTolerance = 0.00; //Meters
    public static final double kYTolerance = 0.00; //Meters
    public static final double kThetaTolerance = 0;
    public static double kStoredRadius = 3.9527559/2; // to be configured later
    public static double kDrivebaseRadius = .409;
  }

  public static class FieldPositions {
    //Blue
    public static final Pose2d L17 = new Pose2d(4.019, 2.913, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d L18 = new Pose2d(3.357, 3.829, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d L19 = new Pose2d(3.75, 5.100, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d L20 = new Pose2d(5.142, 5.263, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d L21 = new Pose2d(5.669, 3.664, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d L22 = new Pose2d(5.142, 3.148, new Rotation2d(Math.toRadians(210)));

    public static final Pose2d R17 = new Pose2d(4.328, 2.764, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d R18 = new Pose2d(3.304, 3.510, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d R19 = new Pose2d(3.439, 4.923, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d R20 = new Pose2d(4.657, 5.436, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d R21 = new Pose2d(5.679, 4.479, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d R22 = new Pose2d(5.483, 3.261, new Rotation2d(Math.toRadians(210)));

    //Red
    public static final Pose2d L6 = new Pose2d(13.65, 2.92, new Rotation2d(Math.toRadians(210)));
    public static final Pose2d L7 = new Pose2d(14.32, 4.00, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d L8 = new Pose2d(13.72, 5.12, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d L9 = new Pose2d(12.45, 5.16, new Rotation2d(Math.toRadians(120)));// Updated
    public static final Pose2d L10 = new Pose2d(11.78, 4.08, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d L11 = new Pose2d(12.38, 2.96, new Rotation2d(Math.toRadians(150)));

    public static final Pose2d R6 = new Pose2d(14.14, 3.20, new Rotation2d(Math.toRadians(210)));
    public static final Pose2d R7 = new Pose2d(14.32, 4.57, new Rotation2d(Math.toRadians(-90)));
    public static final Pose2d R8 = new Pose2d(13.22, 5.40, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d R9 = new Pose2d(11.95, 4.87, new Rotation2d(Math.toRadians(30)));
    public static final Pose2d R10 = new Pose2d(11.78, 3.51, new Rotation2d(Math.toRadians(90)));
    public static final Pose2d R11 = new Pose2d(12.87, 2.67, new Rotation2d(Math.toRadians(150)));
    public static final Pose2d BARGE_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
    public static final Pose2d PROCESSOR_START = new Pose2d(7.449, 6.134, new Rotation2d(Math.toRadians(-30)));
    public static final List<Pose2d> kLeftReefPoses=Arrays.asList(
    L17, L18, L19, L20, L21, L22, L6, L7, L8, L9, L10, L11
    );
    public static final List<Pose2d> kRightReefPoses = Arrays.asList(
      R17, R18, R19, R20, R21, R22, R6, R7, R8, R9, R10, R11
    );

    public static final Translation2d BLUE_REEF_CENTER = new Translation2d(4.5, 4);
    public static final Translation2d RED_REEF_CENTER = new Translation2d(13, 4);


    public static final int [] kReefIDs = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    public static boolean isReefID(int id) {
      for(int i: kReefIDs) {
        if(id==i) return true;
      }
      return false;
    }

    public static final Pose2d BLUE_LEFT_CORAL_STATION_PICKUP = new Pose2d(new Translation2d(1.2,7), Rotation2d.fromDegrees(120));
    public static final Pose2d BLUE_CLIMB_AREA = new Pose2d(new Translation2d(7.638,6.174), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_PROCESSOR = new Pose2d(new Translation2d(6.332,.52), Rotation2d.fromDegrees(-90));
    public static class StartPositions {
      public static final Pose2d ODO_TEST = new PathPlannerAuto("Odom Testing").getStartingPose();
    }
  }
}