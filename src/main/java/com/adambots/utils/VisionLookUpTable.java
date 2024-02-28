package com.adambots.utils;

import java.util.Collections;
import java.util.List;

public class VisionLookUpTable {
    ShooterConfig shooterConfig;

    private static VisionLookUpTable instance = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return instance;
    }
    public VisionLookUpTable() {
        shooterConfig = new ShooterConfig();
        shooterConfig.getShooterConfigs().add(new ShooterPreset(1, 50, 40, 1)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(10, 50, 40, 1.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(21, 50, 40, 2)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(25, 50, 40, 2.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(29, 70, 60, 3)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(32, 70, 60, 3.5)); 
        shooterConfig.getShooterConfigs().add(new ShooterPreset(34.35, 70, 60, 4)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(35.85, 70, 60, 4.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(36.7, 70, 60, 5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(38.58, 70, 60, 5.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(39.6, 70, 60, 6)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(41, 80, 60, 6.5)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(42, 80, 60, 7)); // Distance -> Bumper
        shooterConfig.getShooterConfigs().add(new ShooterPreset(42.8, 80, 60, 7.5)); // Distance -> Bumper

        Collections.sort(shooterConfig.getShooterConfigs());
    }

    /*
     * Obtains a shooter preset from a given target distance
     * @param DistanceFromTarget measured distance to the shooting target
     * @return new shooter preset for given distance
     */
    public ShooterPreset getShooterPreset(double distanceFromTarget) {
        int endIndex = shooterConfig.getShooterConfigs().size()-1;

        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured distance is shorter
         * select the lookup table entry with the shortest distance
         */
        if(distanceFromTarget <= shooterConfig.getShooterConfigs().get(0).getDistance()){
            return shooterConfig.getShooterConfigs().get(0);
        }

        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured distance is larger
         * select the lookup table entry with the largest distance
         */
        if(distanceFromTarget >= shooterConfig.getShooterConfigs().get(endIndex).getDistance()){
            return shooterConfig.getShooterConfigs().get(endIndex);
        }
        /*
         * If the measured distance falls somewhere within the lookup table perform a binary seqarch within the lookup
         * table
         */
        return binarySearchDistance(shooterConfig.getShooterConfigs(),0, endIndex, distanceFromTarget);
    }

    /*
     * Perform fast binary search to find a matching shooter preset. if no matching preset is found it interpolates a
     * new shooter preset based on the two surrounding table entries.
     * 
     * @param ShooterConfigs: the table containing the shooter presets
     * @param StartIndex: Starting point to search
     * @param EndIndex: Ending point to search
     * @param Distance: Distance for which we need to find a preset
     * 
     * @return (Interpolated) shooting preset
     */
    private ShooterPreset binarySearchDistance(List<ShooterPreset> shooterConfigs, int startIndex, int endIndex, double distance) {
        int mid = startIndex + (endIndex - startIndex) / 2;
        double midIndexDistance = shooterConfigs.get(mid).getDistance();

        // If the element is present at the middle
        // return itself
        if (distance == midIndexDistance) {
            return shooterConfigs.get(mid);
        }
        // If only two elements are left
        // return the interpolated config
        if (endIndex - startIndex == 1) {
            double percentIn = (distance - shooterConfig.getShooterConfigs().get(startIndex).getDistance()) / 
                (
                    shooterConfig.getShooterConfigs().get(endIndex).getDistance() - 
                        shooterConfig.getShooterConfigs().get(startIndex).getDistance()
                );
            return interpolateShooterPreset(shooterConfig.getShooterConfigs().get(startIndex), shooterConfig.getShooterConfigs().get(endIndex), percentIn);
        }
        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (distance < midIndexDistance) {
            return binarySearchDistance(shooterConfigs, startIndex, mid, distance);
        }
        // Else the element can only be present in right subarray
        return binarySearchDistance(shooterConfigs, mid, endIndex, distance);
    }

    /*
     * Obtain a new shooter preset by interpolating between two existing shooter presets
     * 
     * @param StartPreset: Starting preset for interpolation
     * @param EndPreset: Ending preset for interpolation
     * @param PercentIn: Amount of percentage between the two values the new preset needs to be
     * 
     * @return new interpolated shooter preset
     */
    private ShooterPreset interpolateShooterPreset(ShooterPreset startPreset, ShooterPreset endPreset, double percentIn) {
        double armAngle = startPreset.getArmAngle() + (endPreset.getArmAngle() - startPreset.getArmAngle()) * percentIn;
        double wristAngle = startPreset.getWristAngle() + (endPreset.getWristAngle() - startPreset.getWristAngle()) * percentIn;
        double shootingSpeed = startPreset.getShootingSpeed() + (endPreset.getShootingSpeed() - startPreset.getShootingSpeed()) * percentIn;
        double distance = startPreset.getDistance() + (endPreset.getDistance() - startPreset.getDistance()) * percentIn;

        return new ShooterPreset(armAngle, wristAngle, shootingSpeed, distance);
    }

    /*
     * MAKE SURE YOU SORT THE LIST BEFORE CALLING THIS FUNCTION
     * @param pShooterConfig a sorted shooter config
     */
    public void setShooterConfig(ShooterConfig pShooterConfig) {
        this.shooterConfig = pShooterConfig;
    }
}
