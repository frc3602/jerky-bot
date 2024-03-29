package frc.team3602.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.lib.util.CANOptimization;
import frc.team3602.robot.Constants;
import frc.team3602.robot.SwerveModule;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  private AHRS navX;

  public Swerve() {
      try {
      navX = new AHRS(SPI.Port.kMXP);
    } catch (RuntimeException ex) {
      System.out.println("ERROR: Unable to instantiate navX" + ex.getMessage());
    }
    
    zeroGyro();

    mSwerveMods = new SwerveModule[] {new SwerveModule(0, Constants.Swerve.Mod0.constants),
        new SwerveModule(1, Constants.Swerve.Mod1.constants),
        new SwerveModule(2, Constants.Swerve.Mod2.constants),
        new SwerveModule(3, Constants.Swerve.Mod3.constants)};

    /*
     * By pausing init for a second before setting module offsets, we avoid a bug with inverting
     * motors. See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
     */
    Timer.delay(1.0);
    resetModulesToAbsolute();

    swerveOdometry =
        new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative,
      boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(),
                rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public void zeroGyro() {
    navX.reset();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - navX.getYaw())
        : Rotation2d.fromDegrees(navX.getYaw());
  }

  public Rotation2d getRoll() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - navX.getRoll())
        : Rotation2d.fromDegrees(navX.getRoll());
  }

  public Rotation2d getPitch() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - navX.getPitch())
        : Rotation2d.fromDegrees(navX.getPitch());
  }

  public void resetModulesToAbsolute() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());

    // for (SwerveModule mod : mSwerveMods) {
    // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder",
    // mod.getCanCoder().getDegrees());
    // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated",
    // mod.getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
    // mod.getState().speedMetersPerSecond);
    // }
  }
}
