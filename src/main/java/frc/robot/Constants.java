// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.simulation.SimVisionSystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
        
        public static class OperatorConstants {
                public static final int kDriverControllerPort = 0;
        }

        public static class DriveConstants{

                public static final double speedMultiplier = 1;
                public static final int[] LeftMotorCAN = {31, 32};
                public static final int[] RightMotorCAN = {33, 34};

                public static final int GyroCAN = 1;
                public static final int PigeonCAN = 30;

                public static final double WheelDiameterMeters = 0.102;

                public static final double gearRatio = 10.71;

                
                public static final double kMaxSpeed = 3.0; //m/s
                public static final double kMaxAngularSpeed = 2 * Math.PI; //one rotation per second
                public static final double kWheelRadius = WheelDiameterMeters/2;
                public static final double kTrackWidth = 0.5; //Horizontal distance btwn wheels
                public static final int kEncoderResolution = 42; //ticks per encoder pulse


  }

  public static class IOConstants{
    public static final int kA = 1, kB = 2, kX = 3, kY = 4, kLB = 5, kRB = 6;
    public static final int kLX = 0, kLY = 1, kRX = 2, kRY = 3;
  }

}
