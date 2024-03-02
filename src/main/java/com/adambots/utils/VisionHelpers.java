
package com.adambots.utils;

import com.adambots.Constants.VisionConstants;
import com.adambots.utils.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;


public class VisionHelpers {
    //private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private VisionHelpers() {}

    private static boolean targetApril = false;
    
    public static void blinkLight(String limelight) {
        LimelightHelpers.setLEDMode_ForceBlink(limelight);
    }
    
     public static int getHeatbeat() {
        // return (int) LimelightHelpers.getFiducialID(VisionConstants.aprilLimelite);
        return (int) NetworkTableInstance.getDefault().getTable(VisionConstants.noteLimelite).getEntry("hb").getDouble(0);
        // return null;
    }

    public static void setApril(boolean value){
        targetApril = value;
    }

    public static boolean getApril(){
        return targetApril;
    }

    public static Pose3d getCameraPose2eTargetSpace() {
        return LimelightHelpers.getBotPose3d_TargetSpace(VisionConstants.aprilLimelite);
    }

    public static double getAprilDistance(){
        return Math.hypot(getCameraPose2eTargetSpace().getX(), getCameraPose2eTargetSpace().getZ());
    }

    public static void offLight(String limelight) {
        LimelightHelpers.setLEDMode_ForceOff(limelight);
    }

    public static Pose3d getCameraPoseTargetSpace(String limelight) {
        return LimelightHelpers.getBotPose3d_TargetSpace(limelight);
    }

    public static Pose2d getAprilTagPose2d() {
        return LimelightHelpers.getLatestResults(VisionConstants.aprilLimelite).targetingResults.getBotPose2d_wpiRed();
    }

    public static int getAprilTagID() {
        // return (int) LimelightHelpers.getFiducialID(VisionConstants.aprilLimelite);
        return (int) NetworkTableInstance.getDefault().getTable(VisionConstants.aprilLimelite).getEntry("tid").getDouble(0);
        // return null;
    }

    public static Pose2d getAprilTagBotPose2d() {
        return LimelightHelpers.getLatestResults(VisionConstants.aprilLimelite).targetingResults.getBotPose2d();  
    }

    public static String getClassName(String limelight) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        if (targetDetector == null || targetDetector.length == 0) {
            return "";
        }

        return targetDetector[0].className;
    }

    public static boolean isDetectingPieces(String limelight) {
        if (getClassName(limelight) == "") {
            return false;
        } else {
            return true;
        }
    }

    public static boolean isDetectingPieces(String limelight, String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;
        
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

    public static double getXLocation(String limelight) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        if (targetDetector == null || targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].tx_pixels;
    }

    public static double getXLocation(String limelight, String type) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

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

    public static double getYLocation(String limelight) {
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

        if (targetDetector == null || targetDetector.length == 0) {
            return -1.0;
        }

        return targetDetector[0].ty_pixels;
    }

    public static double getYLocation(String limelight, String type){
        LimelightTarget_Detector[] targetDetector = LimelightHelpers.getLatestResults(limelight).targetingResults.targets_Detector;

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

    // public static double getDistanceToObject() {
    //     // how many degrees back is your limelight rotated from perfectly vertical?
    //     double limelightMountAngleDegrees = 0.1; //25.0

    //     // distance from the center of the Limelight lens to the floor
    //     double limelightLensHeightInches = 4;

    //     // distance from the target to the floor
    //     double goalHeightInches = 4;

    //     double angleToGoalDegrees = limelightMountAngleDegrees + getYLocation();
    //     // calculate distance
    //     return (goalHeightInches - limelightLensHeightInches) / Math.tan(Math.toRadians(angleToGoalDegrees));
    // }

    public static boolean isAligned(String limelight){
        return Math.abs(getHorizAngle(limelight)) < 5 && isDetected(limelight);
    }

    public static boolean isDistanceAligned(String limelight){
        return getGamePieceArea(limelight) > 10 && getGamePieceArea(limelight) < 15;
    }

    public static void setPipeline(String limelight,int pipeline) {
        LimelightHelpers.setPipelineIndex(limelight, pipeline); 
    }

    public static double getHorizAngle(String limelight) {
        return LimelightHelpers.getTX(limelight);
    }

    public static double getVertAngle(String limelight) {
        return LimelightHelpers.getTY(limelight);
    }

    public static double getGamePieceArea(String limelight) {
        return LimelightHelpers.getTA(limelight);
    }

    public static boolean isDetected(String limelight) {
        return LimelightHelpers.getTV(limelight);
    }
}