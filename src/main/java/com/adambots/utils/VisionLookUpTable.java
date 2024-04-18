package com.adambots.utils;

import java.util.Collections;
import java.util.List;

public class VisionLookUpTable {
    public static ShooterConfig lowShooterConfig;
    public static ShooterConfig defaultShooterConfig;

    private static VisionLookUpTable instance = new VisionLookUpTable();

    public static VisionLookUpTable getInstance() {
        return instance;
    }
    public VisionLookUpTable() {
        lowShooterConfig = new ShooterConfig(); //Lower Angle -> Shoot Higher
        defaultShooterConfig = new ShooterConfig();

        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 310, 90, 1));
        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 313, 90, 1.5));
        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 317, 90, 1.75));
        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 320, 90, 2));
        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 325, 90, 2.5));
        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 330, 90, 3));
        lowShooterConfig.getShooterConfigs().add(new ShooterPreset(125, 335, 90, 3.5));

        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(162, 200, 90, 1.4));
        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(162, 190, 90, 2));
        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(162, 181, 90, 2.5));
        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(162, 176, 90, 3));
        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(162, 170, 90, 3.5));
        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(160, 165, 90, 4));
        defaultShooterConfig.getShooterConfigs().add(new ShooterPreset(160, 162, 90, 4.5));

        Collections.sort(lowShooterConfig.getShooterConfigs());
        Collections.sort(defaultShooterConfig.getShooterConfigs());
    }

    /*
     * Obtains a shooter preset from a given target distance
     * @param DistanceFromTarget measured distance to the shooting target
     * @return new shooter preset for given distance
     */
    public static ShooterPreset getShooterPreset(ShooterConfig selectedShooterConfig, double distanceFromTarget) {
        int endIndex = selectedShooterConfig.getShooterConfigs().size()-1;

        /*
         * Check if distance falls below the shortest distance in the lookup table. If the measured distance is shorter
         * select the lookup table entry with the shortest distance
         */
        if(distanceFromTarget <= selectedShooterConfig.getShooterConfigs().get(0).getDistance()){
            return selectedShooterConfig.getShooterConfigs().get(0);
        }

        /*
         * Check if distance falls above the largest distance in the lookup table. If the measured distance is larger
         * select the lookup table entry with the largest distance
         */
        if(distanceFromTarget >= selectedShooterConfig.getShooterConfigs().get(endIndex).getDistance()){
            return selectedShooterConfig.getShooterConfigs().get(endIndex);
        }
        /*
         * If the measured distance falls somewhere within the lookup table perform a binary seqarch within the lookup
         * table
         */
        return binarySearchDistance(selectedShooterConfig.getShooterConfigs(), selectedShooterConfig, 0, endIndex, distanceFromTarget);
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
    static private ShooterPreset binarySearchDistance(List<ShooterPreset> shooterConfigs, ShooterConfig selectedShooterConfig, int startIndex, int endIndex, double distance) {
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
            double percentIn = (distance - selectedShooterConfig.getShooterConfigs().get(startIndex).getDistance()) / 
                (
                    selectedShooterConfig.getShooterConfigs().get(endIndex).getDistance() - 
                    selectedShooterConfig.getShooterConfigs().get(startIndex).getDistance()
                );
            return interpolateShooterPreset(selectedShooterConfig.getShooterConfigs().get(startIndex), selectedShooterConfig.getShooterConfigs().get(endIndex), percentIn);
        }
        // If element is smaller than mid, then
        // it can only be present in left subarray
        if (distance < midIndexDistance) {
            return binarySearchDistance(shooterConfigs, selectedShooterConfig, startIndex, mid, distance);
        }
        // Else the element can only be present in right subarray
        return binarySearchDistance(shooterConfigs, selectedShooterConfig, mid, endIndex, distance);
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
    private static ShooterPreset interpolateShooterPreset(ShooterPreset startPreset, ShooterPreset endPreset, double percentIn) {
        double armAngle = startPreset.getArmAngle() + (endPreset.getArmAngle() - startPreset.getArmAngle()) * percentIn;
        double wristAngle = startPreset.getWristAngle() + (endPreset.getWristAngle() - startPreset.getWristAngle()) * percentIn;
        double shootingSpeed = startPreset.getShootingSpeed() + (endPreset.getShootingSpeed() - startPreset.getShootingSpeed()) * percentIn;
        double distance = startPreset.getDistance() + (endPreset.getDistance() - startPreset.getDistance()) * percentIn;

        return new ShooterPreset(armAngle, wristAngle, shootingSpeed, distance);
    }
}
