package com.adambots.utils;

import java.util.ArrayList;
import java.util.List;

public class ShooterConfig {
    private List<ShooterPreset> shooterConfigs;

    public ShooterConfig() {
        shooterConfigs = new ArrayList<ShooterPreset>();
    }

    public ShooterConfig(ArrayList<ShooterPreset> shooterConfigs) {
        this.shooterConfigs = shooterConfigs;
    }

    public List<ShooterPreset> getShooterConfigs() {
        return shooterConfigs;
    }
}
