package frc.robot.subsystems.drivetrain.LimelightIO;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private static final int[] botPose = null;
    private final Limelight io;
    private final Limelight inputs;
    private double[] targetSpace;
    private int tv;
    
    public Limelight(Limelight io, Limelight inputs) {
        this.io = io;
        this.inputs = inputs;
        initialize();
    }

    // disables leds by default
    public void initialize() {
        io.setCamMode(0);
        io.setCamMode(1);
    }

    private void setCamMode(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCamMode'");
    }

    /*
    * checks if there is a trusted target being detected by the limelight
    * getTlong() used to detect whether or not the limelight is using MegaTag
    */ 
    public boolean validTarget() {
        if (inputs.tv > 0 && Limelight.botPose[7] < 1) {
            return false;
        }
        if(getTlong() > 24) {
            return inputs.tv > 0 && getTargetSpaceZ() < 8;
        }
        return inputs.tv > 0 && getTargetSpaceZ() < 2;
    }

    public double getTX() {
        // x translation
        return Limelight.botPose[0];
    }

    public double getTY() {
        // y translation
        return Limelight.botPose[1];
    }

    public double getTZ() {
        // z translation
        return Limelight.botPose[2];
    }

    public double getRX() {
        // yaw
        return Limelight.botPose[3];
    }

    public double getRY() {
        // pitch
        return Limelight.botPose[4];
    }

    public double getRZ() {
        // roll
        return Limelight.botPose[5];
    }

    public double getLatency() {
        // gets latency from limelight network tables (ms)
        return Limelight.botPose[6] / 1000;
    }

    public double getTagCount() {
        return Limelight.botPose[7];
    }

    public double getTargetSpaceZ() {
        return inputs.targetSpace[2];
    }

    public double getTlong() {
        return inputs.getTlong();
    }

    public Matrix<N3, N1> calculateTrust() {
        // }
        // return new Matrix<N3, N1>(N3.instance, N1.instance, vals);
        var tagcount = getTagCount();
        return VecBuilder.fill(
            tagcount >= 2.0 ? 0.5 : 10, 
            tagcount >= 2.0 ? 0.5 : 10, 
            999999
        );
    }

    // returns pose from limelight values in field space
    public Pose2d getPose2d() {
        return new Pose2d(getTX(), getTY(), Rotation2d.fromDegrees(getRZ()));
    }
}
