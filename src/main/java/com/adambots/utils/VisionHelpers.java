
package com.adambots.utils;

import com.adambots.utils.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class VisionHelpers {
    //private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private VisionHelpers() {
    }

    public static String getClassName() {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults("limelight").targetingResults.targets_Detector;

        if (targetDetector == null) {
            return "";
        }

        if (targetDetector.length == 0) {
            return "";
        }

        return targetDetector[0].className;
    }

    public static boolean isDetectingPieces() {

        if (getClassName() == "") {
            return false;
        } else {
            return true;
        }
    }

    public static boolean isDetectingPieces(String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults("limelight").targetingResults.targets_Detector;
        
        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        return index != -1;
    }

    public static double getXLocation() {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults("limelight").targetingResults.targets_Detector;

        if (targetDetector == null) {
            return -1.0;
        }

        if (targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].tx_pixels;
    }

    public static double getXLocation(String type) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults("limelight").targetingResults.targets_Detector;

        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        if(index == -1){
            return -1.0;
        }

        return targetDetector[index].tx;
    }

    public static double getYLocation() {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults("limelight").targetingResults.targets_Detector;

        if (targetDetector == null) {
            return -1.0;
        }

        if (targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].ty_pixels;
    }

    public static double getYLocation(String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers
                .getLatestResults("limelight").targetingResults.targets_Detector;

        int index = -1;
        if(targetDetector != null){
         for(int i = targetDetector.length - 1; i >= 0; i--){
            if(targetDetector[i].className.equals(type)){
                index = i;
            }
         }
        }

        if(index == -1){
            return -1.0;
        }

        return targetDetector[index].ty;
    }

    public static double getDistanceToObject() {
        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 0.1; //25.0

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 4;

        // distance from the target to the floor
        double goalHeightInches = 4;

        double angleToGoalDegrees = limelightMountAngleDegrees + getYLocation();
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }
    public static boolean isAligned(){
        if((getHorizAngle()>-3&&getHorizAngle()<3)&&isDetected()){
            return true; 
        }
        return false;
    }
    public static void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex("limelight", pipeline); 
    }

    public static double getHorizAngle() {
        return LimelightHelpers.getTX("limelight");
    }

    public static double getVertAngle() {
        return LimelightHelpers.getTY("limelight");
    }

    public static boolean isDetected() {
        return LimelightHelpers.getTV("limelight");
    }

    public static double getTurnAngle() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue().getRotation().getDegrees();
    }

    public static double getDetectedResult() {
        return LimelightHelpers.getFiducialID("limelight");
    }

    public static Field2d getAprilTagField2d() {
        // return VisionProcessingSubsystem.getAprilField2d();
        // Field2d field = new Field2d();

        // field.setRobotPose(
        //         LimelightHelpers.getLatestResults(VisionConstants.Limelight1).targetingResults.getBotPose2d_wpiBlue());
        // return field;
        return null;

    }

    public static Pose2d getAprilTagPose2d() {
        return LimelightHelpers.getLatestResults("limelight").targetingResults.getBotPose2d_wpiBlue();
    }

    public static double[] getAprilTagArrayPos() {
        return LimelightHelpers.getLimelightNTDoubleArray("limelight", "botpose");
    }
}