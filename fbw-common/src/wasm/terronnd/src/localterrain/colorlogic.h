#pragma once

#include <array>
#include <cstdint>

#include "terrainmap.h"  // for WaterElevation / UnknownElevation / InvalidElevation sentinels

// Ported 1:1 from flybywiresim/simbridge's
// apps/server/src/terrain/processing/gpu/rendering/navigationdisplay.ts and
// apps/server/src/terrain/processing/navigationdisplayrenderer.ts, so a locally-rendered frame uses the exact
// same EGPWS colour/threshold rules a SimBridge-rendered A380X frame would. The GPU-kernel plumbing (gpu.js)
// doesn't carry over -- this is the pure per-pixel/per-frame math the kernels ran, as ordinary scalar code.
//
// One deliberate simplification: simbridge dithers density bands using a ~1.2MB precomputed per-pixel pattern
// texture (gpu/patterns/{arcmode,scanlinemode}.ts) baked so that `patternValue % patternIndex === 0` reproduces
// a stable dot pattern at various fixed densities (patternIndex in {3, 5, 7, 21, 35, 105} -> 1/3, 1/5, 1/7, ...
// of pixels lit). Embedding that literal table isn't practical here, so drawDensityPixel() below reproduces the
// same *density* semantics with a stable per-pixel hash instead of the baked table -- the dot density at each
// threshold matches exactly, the specific dot placement does not.

namespace localterrain {

struct Rgba {
  std::uint8_t r, g, b, a;
};

static constexpr Rgba kTransparent = {4, 4, 5, 0};

static constexpr int HistogramBinRange = 100;
static constexpr int HistogramMinimumElevation = -500;
static constexpr int HistogramMaximumElevation = 29040;
static constexpr int HistogramBinCount = (HistogramMaximumElevation - HistogramMinimumElevation + 1 + HistogramBinRange - 1) / HistogramBinRange;

static constexpr double RenderingLowerPercentile = 0.85;
static constexpr double RenderingUpperPercentile = 0.95;
static constexpr double RenderingFlatEarthThreshold = 100.0;
static constexpr double RenderingNormalModeLowDensityGreenOffset = 2000.0;
static constexpr double RenderingNormalModeHighDensityGreenOffset = 1000.0;
static constexpr double RenderingNormalModeHighDensityYellowOffset = 1000.0;
static constexpr double RenderingNormalModeHighDensityRedOffset = 2000.0;
static constexpr double RenderingGearDownOffset = 250.0;
static constexpr double RenderingNonGearDownOffset = 500.0;
static constexpr double RenderingCutOffAltitudeMinimum = 200.0;
static constexpr double RenderingCutOffAltitudeMaximum = 400.0;
static constexpr double RenderingMaxAirportDistanceNm = 4.0;
static constexpr double FeetPerNauticalMile = 6076.12;
static constexpr double ThreeNauticalMilesInFeet = 18228.3;

/**
 * @brief Stand-in for simbridge's baked dither pattern texture: reproduces the same 1/patternIndex density
 * semantics via a stable per-pixel hash instead of a literal lookup table (see file header comment).
 */
inline Rgba drawDensityPixel(int x, int y, int patternIndex, Rgba color) {
  const unsigned h = static_cast<unsigned>(x * 928371 + y * 1373 * patternIndex);
  const unsigned bucket = h % static_cast<unsigned>(patternIndex);
  return bucket == 0 ? color : kTransparent;
}

struct NormalModeGreenThresholds {
  double lowDensityGreen;
  double highDensityGreen;
};

inline NormalModeGreenThresholds calculateNormalModeGreenThresholds(double referenceAltitude, double minimumElevation,
                                                                     double flatEarth, double lowerPercentile,
                                                                     double halfElevation) {
  NormalModeGreenThresholds t{};

  t.lowDensityGreen = (referenceAltitude - RenderingNormalModeLowDensityGreenOffset <= minimumElevation)
                          ? minimumElevation + 200.0
                          : referenceAltitude - RenderingNormalModeLowDensityGreenOffset;

  t.highDensityGreen = (referenceAltitude - RenderingNormalModeHighDensityGreenOffset <= minimumElevation)
                           ? minimumElevation + 200.0
                           : referenceAltitude - RenderingNormalModeHighDensityGreenOffset;

  if (flatEarth >= 0.0) {
    if (halfElevation <= lowerPercentile && t.lowDensityGreen > halfElevation) {
      t.lowDensityGreen = halfElevation;
    } else if (halfElevation > lowerPercentile && t.lowDensityGreen > lowerPercentile) {
      t.lowDensityGreen = lowerPercentile;
    }
  }

  return t;
}

struct NormalModeWarningThresholds {
  double lowDensityYellow;
  double highDensityYellow;
  double highDensityRed;
};

inline NormalModeWarningThresholds calculateNormalModeWarningThresholds(double referenceAltitude,
                                                                        double minimumElevation,
                                                                        double gearDownAltitudeOffset) {
  NormalModeWarningThresholds t{};
  t.lowDensityYellow = referenceAltitude - gearDownAltitudeOffset;
  t.highDensityYellow = referenceAltitude + RenderingNormalModeHighDensityYellowOffset;
  t.highDensityRed = referenceAltitude + RenderingNormalModeHighDensityRedOffset;

  if (t.lowDensityYellow <= minimumElevation) {
    t.lowDensityYellow = minimumElevation + 200.0;
  }

  return t;
}

struct PeaksModeThresholds {
  double lowerDensity;
  double higherDensity;
  double solidDensity;
};

inline PeaksModeThresholds calculatePeaksModeThresholds(double lowerPercentile, double upperPercentile,
                                                         double halfElevation, double minimumElevation,
                                                         double maximumElevation) {
  PeaksModeThresholds t{};
  t.lowerDensity = lowerPercentile < halfElevation ? lowerPercentile : halfElevation;
  t.higherDensity = upperPercentile < (maximumElevation - minimumElevation) * 0.65 + minimumElevation
                        ? upperPercentile
                        : (maximumElevation - minimumElevation) * 0.65 + minimumElevation;
  t.solidDensity = (maximumElevation - minimumElevation) * 0.95 + minimumElevation;

  if (t.lowerDensity >= t.higherDensity || t.lowerDensity >= t.solidDensity || t.higherDensity >= t.solidDensity ||
      lowerPercentile >= upperPercentile || lowerPercentile >= t.solidDensity || upperPercentile >= t.solidDensity) {
    t.higherDensity = maximumElevation + 100.0;
    t.solidDensity = maximumElevation + 100.0;
  }

  return t;
}

/**
 * @brief Colours one pixel in normal mode (aircraft above the lowest terrain by more than the gear-down
 * offset). elevation/patternX/patternY as for renderPeaksMode(); the remaining parameters are frame-level
 * values computed once per frame (see LocalNdRenderer).
 */
inline Rgba renderNormalMode(float elevation, int patternX, int patternY, double referenceAltitude,
                              double minimumElevation, double flatEarth, double gearDownAltitudeOffset,
                              double lowerPercentileElevation, double halfElevation, double absoluteCutOffAltitude) {
  const auto warning = calculateNormalModeWarningThresholds(referenceAltitude, minimumElevation, gearDownAltitudeOffset);
  const auto green =
      calculateNormalModeGreenThresholds(referenceAltitude, minimumElevation, flatEarth, lowerPercentileElevation, halfElevation);

  if (elevation != InvalidElevation && elevation != UnknownElevation && elevation != WaterElevation &&
      elevation >= absoluteCutOffAltitude) {
    if (elevation >= warning.highDensityRed) {
      return drawDensityPixel(patternX, patternY, 5, {255, 0, 0, 255});
    }
    if (elevation >= warning.highDensityYellow) {
      return drawDensityPixel(patternX, patternY, 5, {255, 255, 50, 255});
    }
    if (elevation >= green.highDensityGreen && elevation < warning.lowDensityYellow) {
      return drawDensityPixel(patternX, patternY, 5, {0, 255, 0, 255});
    }
    if (elevation >= warning.lowDensityYellow && elevation < warning.highDensityYellow) {
      return drawDensityPixel(patternX, patternY, 3, {255, 255, 50, 255});
    }
    if (elevation >= green.lowDensityGreen && elevation < green.highDensityGreen) {
      return drawDensityPixel(patternX, patternY, 3, {0, 255, 0, 255});
    }
  } else if (elevation == WaterElevation) {
    return drawDensityPixel(patternX, patternY, 7, {0, 255, 255, 255});
  } else if (elevation == UnknownElevation) {
    return drawDensityPixel(patternX, patternY, 5, {255, 148, 255, 255});
  }

  return {0, 0, 0, 255};
}

/**
 * @brief Colours one pixel in peaks mode (aircraft not clearly above the lowest terrain).
 */
inline Rgba renderPeaksMode(float elevation, int patternX, int patternY, double lowerPercentile, double upperPercentile,
                             double halfElevation, double minimumElevation, double maximumElevation) {
  const auto thresholds = calculatePeaksModeThresholds(lowerPercentile, upperPercentile, halfElevation, minimumElevation, maximumElevation);

  if (elevation != InvalidElevation && elevation != UnknownElevation && elevation != WaterElevation) {
    if (thresholds.solidDensity <= elevation) {
      return {0, 255, 0, 255};
    }
    if (thresholds.higherDensity <= elevation) {
      return drawDensityPixel(patternX, patternY, 5, {0, 255, 0, 255});
    }
    if (thresholds.lowerDensity <= elevation) {
      return drawDensityPixel(patternX, patternY, 3, {0, 255, 0, 255});
    }
  } else if (elevation == WaterElevation) {
    return drawDensityPixel(patternX, patternY, 7, {0, 255, 255, 255});
  } else if (elevation == UnknownElevation) {
    return drawDensityPixel(patternX, patternY, 5, {255, 148, 255, 255});
  }

  return {0, 0, 0, 255};
}

}  // namespace localterrain
