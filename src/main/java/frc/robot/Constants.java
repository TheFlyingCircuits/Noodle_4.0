// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final static boolean atCompetition = false;

    public final class UniversalConstants {
        public final static double gravityMetersPerSecondSquared = 9.81;
        public final static double defaultPeriodSeconds = 0.02;

        // public final static String canivoreName = "CTRENetwork";

        public enum Direction {
            left,
            right
        }

    }


    public final static class DrivetrainConstants {
         // KINEMATICS CONSTANTS

        /**
         * Distance between the center point of the left wheels and the center point of the right wheels.
         */

        public static final double trackwidthMeters = Units.inchesToMeters(22.75);
        /**
         * Distance between the center point of the front wheels and the center point of
         * the back wheels.
         */
        public static final double wheelbaseMeters = Units.inchesToMeters(22.75);

        public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
        public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final double wheelDiamaterMeters = 0.10033;
        public static final double wheelCircumferenceMeters = wheelDiamaterMeters * Math.PI;
        public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0);

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
                new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
                new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
                new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));
        public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = Units.rotationsToRadians(0.85);
         
        // public static final double maxAchievableVelocityMetersPerSecond = 5880.0 / 60.0 *
        // driveReduction *
        // wheelDiamaterMeters * Math.PI;
        public static final double maxAchievableVelocityMetersPerSecond = 2.5;
        public static final double maxDesiredTeleopVelocityMetersPerSecond = 4.3;
        public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond /
                Math.hypot(trackwidthMeters / 2.0, wheelbaseMeters / 2.0);
        public static final double maxDesiredAngularVelocityRadiansPerSecond = 5.4;

        public static final double frameWidthMeters = Units.inchesToMeters(27);
        // TODO: get real values for framWidth and bumperWidth
        public static final double bumperWidthMeters = Units.inchesToMeters(27 + 7);

        public static final double maxDesiredDriverAccel = 27.27;
        public static final PathConstraints pathfindingConstraints = new PathConstraints(
                1.0, 1.0,
                Units.degreesToRadians(360), Units.degreesToRadians(360));
    }

    public final static class ControllerConstants {
        public static final double controllerDeadzone = 0.075;
        public static final double maxThrottle = 1.0;
    }


    public final static class SwerveModuleConstants {
      /** Rotations of the drive wheel per rotations of the drive motor. */
      public static final double driveGearReduction = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

      /** Rotations of the steering column per rotations of the angle motor. */
      public static final double steerGearReduction = (14.0 / 50.0) * (10.0 / 60.0);

        public static final int angleContinuousCurrentLimit = 50;
        public static final boolean angleInvert = true;

        public static final int driveContinuousCurrentLimit = 60;
        public static final boolean driveInvert = true;

      // The wheels have a 2 inch radius, but sink into the capet about (1/16) of an inch.
      // As an estimate, the wheel radius is Units.inchesToMeters(2.-1./16.), or 0.0492m
      public static final double wheelRadiusMeters = 0.04946; //use MeasureWheelDiameter for this!
      public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters; // ~0.31

      public static final double drivekPVoltsPerMeterPerSecond = 0.1;
      public static final double drivekIVoltsPerMeter = 0.;
      public static final double drivekDVoltsPerMeterPerSecondSquared = 0.;

      // PID for angle motors.
      public static final double anglekPVoltsPerDegree = 0.08;
      public static final double anglekIVoltsPerDegreeSeconds = 0.; // this might be the wrong unit idk 
      public static final double anglekDVoltsPerDegreePerSecond = 0.;

      public static final double drivekSVolts = 0.2383;
      public static final double drivekVVoltsSecondsPerMeter = 2.2859;
      public static final double drivekAVoltsSecondsSquaredPerMeter = 0.;

      
    }

    public final static class GyroConstants {
        public static final int pigeonID = 0;


        //Follow the mount calibration process in Phoenix Tuner to determine these
        public static final double mountPoseYawDegrees = 0.7333114147186279;
        public static final double mountPosePitchDegrees = -0.11852765083312988;
        public static final double mountPoseRollDegrees = -1.0425487756729126;
    }

    public final static class VisionConstants {

        public final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);;
                                                       
        public final static String[] tagCameraNames = {

        };
        public final static Transform3d robotToCoralCamera = new Transform3d(
            new Translation3d(Units.inchesToMeters(-9.75), Units.inchesToMeters(5.5), Units.inchesToMeters(26.)),
            new Rotation3d(0, Math.toRadians(19), Math.toRadians(-12))
        );

        public final static Transform3d[] tagCameraTransforms = {

        };


    }

    public final static class IntakeConstants {
        public final static int leftPivotNeoID = 0;
        public final static int rightPivotNeoID = 0;
        public final static int leftGripperNeoID = 0;
        public final static int rightGripperNeoID = 0;

        public final static int pivotGearReduction = 1/2; // TODO: change for real robot, also make sure it is a fraction or decimal

        public final static double kSPivotVolts = 0;
        public final static double kGPivotVolts = 0;
        public final static double kVPivotVoltsSecondsPerRadian = 0;
        public final static double kAPivotVoltsSecondsSquaredPerRadian = 0;

        //in deg bc simplier and feedforward in the end just outputs a voltage so we can use rad for that and deg for this
        public final static double kPPivotVoltsPerRadian = 0;
        public final static double kDPivotVoltsPerRadianPerSecond = 0;

        // neo max free spinning speed at nominal voltage is 5820 rpm
        // this is theoretical max velocity
        public final static double maxPivotVelocityRadianPerSecond = 2*Math.PI * (5820/60) * pivotGearReduction; // make rpm rps by /60 then div by gear ratio then converting it to radians by 2pi*
        public final static double maxPivotAccelerationRadianPerSecondSquared = 0; // idk will get slope of velocity vs time graph on real mechanism running at nominal voltage
        
    }

    public final static class LEDConstants {
        public final static int ledPWMPort = 0;

        //total number of leds
        public final static int ledsPerStrip = 60;
        

        public final static double stripLengthMeters = 1.0;

        public final static double ledsPerMeter = (1.0 * ledsPerStrip) / stripLengthMeters;

        public final static double metersPerLed = 1/ledsPerMeter;

        /**
         * Hues for specific colors
         * Values use the openCV convention where hue ranges from [0, 180)
         */
        public final static class Hues {

            public final static int orangeSignalLight = 4;
            public final static int blueBumpers = 114;
            public final static int redBumpers = 0;
            public final static int redTrafficLight = 0;//0;
            public final static int greenTrafficLight = 40;//60;
            public final static int betweenBlueAndRed = 150; // a purple/pink that's between blue and red.

        }
    }

}
