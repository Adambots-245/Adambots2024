// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Source: Team 1596

package com.adambots.utils;

import java.util.*;

import com.adambots.Constants.DriveConstants.ModulePosition;
import com.adambots.subsystems.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Contains functions to convert {@link Map}s with {@link ModulePosition} keys
 * to and from arrays so
 * that it's easier to use WPILib swerve functions.
 */
public class ModuleMap {

  /**
   * Creates a {@code Map} with {@link ModulePosition} keys from multiple values,
   * in the order
   * specified in the {@link ModulePosition} enum.
   *
   * <p>
   * For processing the output of a WPILib swerve function which returns an array.
   *
   * @param values
   *          Must have at least as many elements as {@link ModulePosition}
   *          has entries. Any
   *          entries after will be ignored.
   */
  @SafeVarargs
  public static <V> Map<ModulePosition, V> of(V... values) {
    Map<ModulePosition, V> map = new HashMap<>();
    for (int i = 0; i < ModulePosition.values().length; i++) {
      map.put(ModulePosition.values()[i], values[i]);
    }
    return map;
  }

  /**
   * Returns the values from a map as a {@link List} in the same order as in the
   * {@link
   * ModulePosition} enum.
   *
   * <p>
   * You can use this in a for/in loop without needing to supply an empty array
   * like in {@link
   * #orderedValues(Map, Object[]) orderedValues}.
   */
  public static <V> List<V> orderedValuesList(Map<ModulePosition, V> map) {
    ArrayList<V> list = new ArrayList<>();
    // System.out.println("Map: " + map.size());
    for (ModulePosition i : ModulePosition.values()) {
      // System.out.println(i.name());
      list.add(map.get(i));
    }
    return list;
  }

  /**
   * Returns the values from the map as an {@code Array} in the same order as in
   * the {@link
   * ModulePosition} enum. This avoids any error in the order of the motors and
   * modules.
   *
   * <p>
   * Useful when a WPILib swerve function requires an array as input.
   *
   * @param swerveModulePositions
   *          An array of the class to output an array of,
   *          e.g. {@code
   *     moduleTranslations.valuesArray(new SwerveModulePosition[0])}. Required
   *          because
   *          Java can't make an
   *          array of generics.
   */
  public static SwerveModulePosition[] orderedModulePositions(Map<ModulePosition, SwerveModule> map) {

    ArrayList<SwerveModulePosition> list = new ArrayList<>();

    for (ModulePosition i : ModulePosition.values()) {
      list.add(map.get(i).getPosition());
    }

    return (SwerveModulePosition[]) list.toArray(new SwerveModulePosition[0]);
  }

  public static Pose2d orderedValues(Map<ModulePosition, Pose2d> m_swerveModulePoses, Pose2d poser) {
    return null;
  }

  public static SwerveModuleState[] orderedValues(Map<ModulePosition, SwerveModuleState> moduleStates,
      SwerveModuleState[] swerveModuleStates, double kmaxspeedmeterspersecond) {
    return null;
  }

  public static SwerveModuleState[] orderedValues(Map<ModulePosition, SwerveModuleState> moduleStates,
      SwerveModuleState[] swerveModuleStates) {
    return (SwerveModuleState[]) orderedValuesList(moduleStates).toArray(swerveModuleStates);
  }

  public static <V> V[] orderedValues(Map<ModulePosition, V> map, V[] array) {
    return orderedValuesList(map).toArray(array);
  }

  public static Translation2d[] orderedValues(Map<ModulePosition, Translation2d> kmoduletranslations,
      Translation2d[] translation2ds) {
    return (Translation2d[]) orderedValuesList(kmoduletranslations).toArray(translation2ds);
  }

  public static void setDesiredState(HashMap<ModulePosition, SwerveModule> swerveModules, SwerveModuleState[] swerveModuleStates) {

    int idx = 0;
    for (ModulePosition i : ModulePosition.values()) {
      swerveModules.get(i).setDesiredState(swerveModuleStates[idx++]);
    }
  }

  public static SwerveModuleState[] getState(HashMap<ModulePosition, SwerveModule> swerveModules) {
    SwerveModuleState[] arr = new SwerveModuleState[4];

    int idx = 0;
    for (ModulePosition i : ModulePosition.values()) {
      arr[idx++] = swerveModules.get(i).getState();
    }
    return arr;
  }
}
