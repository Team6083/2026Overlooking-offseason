// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.shooting;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import edu.wpi.first.wpilibj.Filesystem;

/**
 * 讀取實測射球網格資料(距離, 角度, 轉速)，並提供查詢
 * CSV 格式: distance_cm,angle_deg,velocity_rpm (第一行不讀取)
 */
public class ShotTable {

  /** 在某角度下命中該距離所需的轉速 */
  public record Candidate(double angleDeg, double velocityRpm) {
  }

  // 依角度分組，每組內是 (距離 -> 轉速) 的排序，方便沿單一角度做距離內插
  private final Map<Double, TreeMap<Double, Double>> byAngle = new TreeMap<>();

  private boolean loaded = false;

  public ShotTable(String deployRelativePath) {
    File file = new File(Filesystem.getDeployDirectory(), deployRelativePath);
    load(file);
  }

  private void load(File file) {
    try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
      String line = reader.readLine(); // skip header
      while ((line = reader.readLine()) != null) {
        line = line.trim();
        if (line.isEmpty()) {
          continue;
        }
        String[] parts = line.split(",");
        if (parts.length < 3) {
          continue;
        }
        double distance = Double.parseDouble(parts[0].trim());
        double angle = Double.parseDouble(parts[1].trim());
        double velocity = Double.parseDouble(parts[2].trim());

        byAngle.computeIfAbsent(angle, k -> new TreeMap<>()).put(distance, velocity);
      }
      loaded = !byAngle.isEmpty();
    } catch (IOException | NumberFormatException e) {
      // 讀檔失敗時保持空表，getCandidates 會回傳空list，呼叫端要自行處理 fallback
      loaded = false;
      System.err.println("ShotTable: 讀取失敗 - " + e.getMessage());
    }
  }

  public boolean isLoaded() {
    return loaded;
  }

  /**
   * 給定距離，回傳所有角度中「有實測資料涵蓋這個距離範圍」的候選解。
   * 若某角度的實測範圍不包含這個距離，該角度會被跳過(不外插)。
   */
  public List<Candidate> getCandidates(double distanceCm) {
    List<Candidate> candidates = new ArrayList<>();

    for (Map.Entry<Double, TreeMap<Double, Double>> entry : byAngle.entrySet()) {
      double angle = entry.getKey();
      TreeMap<Double, Double> distanceToVelocity = entry.getValue();

      Double interpolated = interpolate(distanceToVelocity, distanceCm);
      if (interpolated != null) {
        candidates.add(new Candidate(angle, interpolated));
      }
    }

    return candidates;
  }

  /** 在單一角度的 (距離->轉速) 表中線性內插，超出實測範圍回傳 null(不外插) */
  private Double interpolate(TreeMap<Double, Double> table, double distanceCm) {
    Map.Entry<Double, Double> floor = table.floorEntry(distanceCm);
    Map.Entry<Double, Double> ceiling = table.ceilingEntry(distanceCm);

    if (floor == null || ceiling == null) {
      return null; // 超出這個角度的實測範圍
    }
    if (floor.getKey().equals(ceiling.getKey())) {
      return floor.getValue(); // 剛好打中實測點
    }

    double x0 = floor.getKey();
    double y0 = floor.getValue();
    double x1 = ceiling.getKey();
    double y1 = ceiling.getValue();

    double t = (distanceCm - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }

  /** 在候選解中，挑選轉速與目前飛輪轉速差距最小的一組(省 spin-up 時間) */
  public Candidate pickClosestVelocity(double distanceCm, double currentVelocityRpm) {
    List<Candidate> candidates = getCandidates(distanceCm);
    if (candidates.isEmpty()) {
      return null;
    }
    Candidate best = candidates.get(0);
    double bestDiff = Math.abs(best.velocityRpm() - currentVelocityRpm);
    for (Candidate c : candidates) {
      double diff = Math.abs(c.velocityRpm() - currentVelocityRpm);
      if (diff < bestDiff) {
        best = c;
        bestDiff = diff;
      }
    }
    return best;
  }

  /** 在候選解中，挑選轉速最低的一組(最省電) */
  public Candidate pickLowestVelocity(double distanceCm) {
    List<Candidate> candidates = getCandidates(distanceCm);
    if (candidates.isEmpty()) {
      return null;
    }
    Candidate best = candidates.get(0);
    for (Candidate c : candidates) {
      if (c.velocityRpm() < best.velocityRpm()) {
        best = c;
      }
    }
    return best;
  }
}