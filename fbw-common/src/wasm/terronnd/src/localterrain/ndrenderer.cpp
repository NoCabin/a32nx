#include "ndrenderer.h"

// See the matching comment in terrainmap.cpp -- guarded for the same reason.
#ifdef A380X

#include <algorithm>
#include <cmath>

#include "colorlogic.h"
#include "geomath.hpp"

using namespace localterrain;

namespace {

// Outer range ring radius as a fraction of the ND's 768-wide SVG viewBox width -- see ndrenderer.h's file
// comment. mapWidth is the gauge's own actual reported canvas width, used as the reference scale instead of
// assuming any particular pixel size.
constexpr double kArcRingRadiusFraction = 492.0 / 768.0;
constexpr double kRoseRingRadiusFraction = 250.0 / 768.0;

// ARC mode draws a straight heading tape above the circle (see the tick marks around y=91..128 in
// ArcModeUnderlay.tsx), so the circle's top isn't at the very top of the buffer -- there's a margin for that
// tape first. ROSE mode's compass is the rotating ring itself, no separate tape, so no margin needed.
constexpr double kArcTopMarginFraction = 130.0 / 768.0;

double ringRadiusPixels(const NdFrameConfig& cfg) {
  return cfg.mapWidth * (cfg.arcMode ? kArcRingRadiusFraction : kRoseRingRadiusFraction);
}

double ringTopMarginPixels(const NdFrameConfig& cfg) {
  return cfg.arcMode ? cfg.mapWidth * kArcTopMarginFraction : 0.0;
}

// Aircraft symbol's row, measured down from the top of the buffer.
double ringCenterYPixels(const NdFrameConfig& cfg) {
  return ringTopMarginPixels(cfg) + ringRadiusPixels(cfg);
}

/**
 * @brief Geometric stand-in for SimBridge's baked pattern-texture shape mask (see ndrenderer.h's file
 * comment): ARC mode only shows the forward half-plane out to the outer range ring (nothing behind the
 * aircraft symbol); ROSE mode shows a full circle of the same ring radius.
 */
bool isPixelVisible(const NdFrameConfig& cfg, double dx, double dy, double distancePixels) {
  (void)dx;
  if (cfg.arcMode && dy < 0.0) return false;
  return distancePixels <= ringRadiusPixels(cfg);
}

/**
 * @brief The row range that can possibly contain a visible pixel, clamped to the buffer -- used to skip whole
 * rows of guaranteed-invisible work (e.g. everything below the aircraft in ARC mode) instead of visiting every
 * row of a buffer that may be considerably taller than the visible ring.
 */
void visibleRowRange(const NdFrameConfig& cfg, int* outRowStart, int* outRowEnd) {
  const double centerY = ringCenterYPixels(cfg);
  const double radius = ringRadiusPixels(cfg);

  const double yTop = centerY - radius;
  const double yBottom = cfg.arcMode ? centerY : centerY + radius;

  int rowStart = static_cast<int>(std::floor(yTop));
  int rowEnd = static_cast<int>(std::ceil(yBottom)) + 1;
  if (rowStart < 0) rowStart = 0;
  if (rowEnd > cfg.mapHeight) rowEnd = cfg.mapHeight;
  if (rowStart > rowEnd) rowStart = rowEnd;

  *outRowStart = rowStart;
  *outRowEnd = rowEnd;
}

}  // namespace

void LocalNdRenderer::reset() {
  this->_phase = Phase::Idle;
  this->_fillRow = 0;
  this->_colorPatchRow = 0;
  this->_idleUntilSeconds = 0.0;
  this->_elevationGrid.clear();
  this->_result = NdFrameResult{};
}

void LocalNdRenderer::beginCycle(const NdFrameConfig& config) {
  this->_cycleConfig = config;
  this->_phase = Phase::Filling;
  this->_elevationGrid.assign(static_cast<std::size_t>(config.mapWidth) * static_cast<std::size_t>(config.mapHeight),
                              InvalidElevation);

  visibleRowRange(config, &this->_visibleRowStart, &this->_visibleRowEnd);
  this->_fillRow = this->_visibleRowStart;

  this->_visiblePatchRowStart = this->_visibleRowStart / 8;
  this->_visiblePatchRowEnd = (this->_visibleRowEnd + 7) / 8;
  this->_colorPatchRow = this->_visiblePatchRowStart;
}

void LocalNdRenderer::fillRows(const TerrainMap& terrain, int rowStart, int rowEnd) {
  const NdFrameConfig& cfg = this->_cycleConfig;
  const double centerX = cfg.mapWidth / 2.0;
  const double centerY = ringCenterYPixels(cfg);
  const double radius = ringRadiusPixels(cfg);

  double metresPerPixel = std::round((cfg.rangeNm * 1852.0) / radius);
  if (cfg.arcMode) metresPerPixel *= 2.0;

  for (int y = rowStart; y < rowEnd; ++y) {
    const double dy = centerY - y;

    for (int x = 0; x < cfg.mapWidth; ++x) {
      const double dx = x - centerX;
      const double distancePixels = std::sqrt(dx * dx + dy * dy);
      const std::size_t idx =
          static_cast<std::size_t>(y) * static_cast<std::size_t>(cfg.mapWidth) + static_cast<std::size_t>(x);

      if (!isPixelVisible(cfg, dx, dy, distancePixels)) {
        this->_elevationGrid[idx] = InvalidElevation;
        continue;  // skip the geodesic projection + tile lookup entirely for pixels outside the display shape
      }

      double lat = cfg.aircraftLatitude;
      double lon = cfg.aircraftLongitude;

      if (distancePixels > 1e-6) {
        const double distanceMetres = distancePixels * (metresPerPixel / 2.0);
        const double angle = rad2deg(std::acos(dy / distancePixels));
        double bearing = (x > centerX) ? angle : 360.0 - angle;
        bearing = normalizeHeading(bearing + cfg.headingDeg);

        const LatLon projected = projectWgs84(cfg.aircraftLatitude, cfg.aircraftLongitude, bearing, distanceMetres);
        lat = projected.latitude;
        lon = projected.longitude;
      }

      this->_elevationGrid[idx] = terrain.sampleElevationFt(lat, lon);
    }
  }
}

namespace {

double calculateAbsoluteCutOffAltitude(const NdFrameConfig& cfg, const TerrainMap& terrain) {
  if (!cfg.destinationValid) {
    return HistogramMinimumElevation;
  }

  const float destinationElevation = terrain.sampleElevationFt(cfg.destinationLatitude, cfg.destinationLongitude);
  if (destinationElevation == UnknownElevation) {
    return HistogramMinimumElevation;
  }

  double cutOffAltitude = RenderingCutOffAltitudeMaximum;

  const double distanceNm =
      distanceWgs84Nm(cfg.aircraftLatitude, cfg.aircraftLongitude, cfg.destinationLatitude, cfg.destinationLongitude);
  if (distanceNm <= RenderingMaxAirportDistanceNm) {
    const double distanceFeet = distanceNm * FeetPerNauticalMile;

    const double opposite = cfg.altitudeFt - static_cast<double>(destinationElevation);
    double glideRadian = 0.0;
    if (opposite > 0.0 && distanceNm > 0.0) {
      glideRadian = std::atan(opposite / distanceFeet);
    }

    if (glideRadian < 0.0523599) {
      if (distanceNm <= 1.0 || glideRadian == 0.0) {
        cutOffAltitude = RenderingCutOffAltitudeMinimum;
      } else {
        const double slope = (RenderingCutOffAltitudeMinimum - RenderingCutOffAltitudeMaximum) / ThreeNauticalMilesInFeet;
        cutOffAltitude = std::round(slope * (distanceFeet - FeetPerNauticalMile) + RenderingCutOffAltitudeMaximum);
        cutOffAltitude = std::max(cutOffAltitude, RenderingCutOffAltitudeMinimum);
        cutOffAltitude = std::min(cutOffAltitude, RenderingCutOffAltitudeMaximum);
      }
    }
  }

  return cutOffAltitude;
}

}  // namespace

void LocalNdRenderer::computeThresholds(const TerrainMap& terrain) {
  const NdFrameConfig& cfg = this->_cycleConfig;

  // --- histogram (excludes unknown/invalid/water samples, matching simbridge's createLocalElevationHistogram) ---
  std::vector<int> histogram(static_cast<std::size_t>(HistogramBinCount), 0);
  for (const float elevation : this->_elevationGrid) {
    if (elevation == UnknownElevation || elevation == InvalidElevation || elevation == WaterElevation) continue;
    const double shifted = static_cast<double>(elevation) - HistogramMinimumElevation;
    int bin = static_cast<int>(std::ceil(shifted / HistogramBinRange));
    if (bin < 0) bin = 0;
    if (bin > HistogramBinCount) bin = HistogramBinCount;
    if (bin < HistogramBinCount) histogram[static_cast<std::size_t>(bin)] += 1;
  }

  this->_cutOffAltitude = calculateAbsoluteCutOffAltitude(cfg, terrain);
  const int cutOffAltitudeBin = static_cast<int>(std::floor((this->_cutOffAltitude - HistogramMinimumElevation) / HistogramBinRange));
  this->_referenceAltitude = cfg.altitudeFt + (cfg.verticalSpeedFtMin <= -1000.0 ? cfg.verticalSpeedFtMin * 0.5 : 0.0);

  int totalFrequency = 0;
  for (int bin = std::max(0, cutOffAltitudeBin); bin < HistogramBinCount; ++bin) {
    totalFrequency += histogram[static_cast<std::size_t>(bin)];
  }

  int minElevationBin = -1, maxElevationBin = -1, lowerBin = -1, upperBin = -1;
  double currentPercentile = 0.0;
  for (int bin = std::max(0, cutOffAltitudeBin); bin < HistogramBinCount; ++bin) {
    if (totalFrequency > 0) {
      currentPercentile += static_cast<double>(histogram[static_cast<std::size_t>(bin)]) / totalFrequency;
      if (lowerBin == -1 && currentPercentile >= RenderingLowerPercentile) lowerBin = bin;
      if (upperBin == -1 && currentPercentile >= RenderingUpperPercentile) upperBin = bin;
    }
    if (histogram[static_cast<std::size_t>(bin)] > 0) {
      if (minElevationBin < 0) minElevationBin = bin;
      maxElevationBin = bin;
    }
  }
  if (upperBin < 0) upperBin = HistogramBinCount - 1;

  this->_lowerPercentileElevation = lowerBin * HistogramBinRange + HistogramMinimumElevation;
  this->_upperPercentileElevation = upperBin * HistogramBinRange + HistogramMinimumElevation;

  this->_minElevation = minElevationBin >= 0 ? minElevationBin * HistogramBinRange + HistogramMinimumElevation : -1.0;
  this->_maxElevation = maxElevationBin >= 0 ? (maxElevationBin + 1) * HistogramBinRange + HistogramMinimumElevation : 0.0;

  this->_flatEarth = RenderingFlatEarthThreshold - (this->_maxElevation - this->_minElevation);
  this->_halfElevation = this->_maxElevation * 0.5;
  this->_gearDownAltitudeOffset = cfg.gearIsDown ? RenderingGearDownOffset : RenderingNonGearDownOffset;

  this->_normalMode = this->_maxElevation >= this->_referenceAltitude - this->_gearDownAltitudeOffset;

  this->_result.width = cfg.mapWidth;
  this->_result.height = cfg.mapHeight;
  this->_result.rgba.assign(static_cast<std::size_t>(cfg.mapWidth) * static_cast<std::size_t>(cfg.mapHeight) * 4, 0);
}

void LocalNdRenderer::colorPatchRows(int patchRowStart, int patchRowEnd) {
  const NdFrameConfig& cfg = this->_cycleConfig;
  const double centerX = cfg.mapWidth / 2.0;
  const double centerY = ringCenterYPixels(cfg);

  // --- 8x8-patch max elevation (matches simbridge's blocky EGPWS look), one patch pass then one dither pass
  //     per patch, O(width*height) total rather than O(width*height*64) ---
  for (int patchY = patchRowStart * 8; patchY < std::min(cfg.mapHeight, patchRowEnd * 8); patchY += 8) {
    const int patchYEnd = std::min(cfg.mapHeight, patchY + 8);
    for (int patchX = 0; patchX < cfg.mapWidth; patchX += 8) {
      const int patchXEnd = std::min(cfg.mapWidth, patchX + 8);

      float patchMax = -1000.0f;
      bool anyVisible = false;
      for (int py = patchY; py < patchYEnd; ++py) {
        for (int px = patchX; px < patchXEnd; ++px) {
          const float e = this->_elevationGrid[static_cast<std::size_t>(py) * static_cast<std::size_t>(cfg.mapWidth) +
                                                static_cast<std::size_t>(px)];
          if (e == InvalidElevation) continue;
          anyVisible = true;
          if (e > patchMax) patchMax = e;
        }
      }
      if (!anyVisible) continue;  // whole patch outside the display shape -- leave fully transparent

      for (int py = patchY; py < patchYEnd; ++py) {
        const double dy = centerY - py;
        for (int px = patchX; px < patchXEnd; ++px) {
          const double dx = px - centerX;
          const double distancePixels = std::sqrt(dx * dx + dy * dy);
          if (!isPixelVisible(cfg, dx, dy, distancePixels)) continue;  // leave this one pixel transparent

          const Rgba color = this->_normalMode
                                  ? renderNormalMode(patchMax, px, py, this->_referenceAltitude, this->_minElevation,
                                                      this->_flatEarth, this->_gearDownAltitudeOffset,
                                                      this->_lowerPercentileElevation, this->_halfElevation, this->_cutOffAltitude)
                                  : renderPeaksMode(patchMax, px, py, this->_lowerPercentileElevation,
                                                     this->_upperPercentileElevation, this->_halfElevation, this->_minElevation,
                                                     this->_maxElevation);

          const std::size_t idx =
              (static_cast<std::size_t>(py) * static_cast<std::size_t>(cfg.mapWidth) + static_cast<std::size_t>(px)) * 4;
          this->_result.rgba[idx + 0] = color.r;
          this->_result.rgba[idx + 1] = color.g;
          this->_result.rgba[idx + 2] = color.b;
          this->_result.rgba[idx + 3] = color.a;
        }
      }
    }
  }
}

void LocalNdRenderer::publishThresholds() {
  // matches simbridge's NavigationDisplayRenderer.analyzeMetadata()
  if (this->_normalMode) {
    const auto warning = calculateNormalModeWarningThresholds(this->_referenceAltitude, this->_minElevation, this->_gearDownAltitudeOffset);
    const auto green = calculateNormalModeGreenThresholds(this->_referenceAltitude, this->_minElevation, this->_flatEarth,
                                                            this->_lowerPercentileElevation, this->_halfElevation);

    this->_result.minimumElevationFt = static_cast<float>(std::max(this->_cutOffAltitude, green.lowDensityGreen));
    this->_result.minimumElevationMode =
        (warning.lowDensityYellow <= green.highDensityGreen) ? ThresholdMode::WARNING : ThresholdMode::PEAKS_MODE;
    this->_result.maximumElevationFt = static_cast<float>(this->_maxElevation);
    this->_result.maximumElevationMode = (this->_maxElevation >= warning.highDensityRed) ? ThresholdMode::CAUTION : ThresholdMode::WARNING;
  } else {
    const auto thresholds = calculatePeaksModeThresholds(this->_lowerPercentileElevation, this->_upperPercentileElevation,
                                                          this->_halfElevation, this->_minElevation, this->_maxElevation);

    if (this->_maxElevation < 0.0) {
      this->_result.minimumElevationFt = -1.0f;
      this->_result.maximumElevationFt = 0.0f;
    } else {
      this->_result.minimumElevationFt = static_cast<float>(std::max(thresholds.lowerDensity, this->_minElevation));
      this->_result.maximumElevationFt = static_cast<float>(this->_maxElevation);
    }
    this->_result.minimumElevationMode = ThresholdMode::PEAKS_MODE;
    this->_result.maximumElevationMode = ThresholdMode::PEAKS_MODE;
  }
}

bool LocalNdRenderer::update(const NdFrameConfig& config, const TerrainMap& terrain, double simTimeSeconds) {
  bool configChanged = true;
  if (this->_phase != Phase::Idle || this->_result.width != 0) {
    configChanged = config.mapWidth != this->_cycleConfig.mapWidth || config.mapHeight != this->_cycleConfig.mapHeight ||
                    config.arcMode != this->_cycleConfig.arcMode ||
                    std::abs(config.rangeNm - this->_cycleConfig.rangeNm) > 0.05;
  }

  if (this->_phase == Phase::Idle) {
    if (!configChanged && simTimeSeconds < this->_idleUntilSeconds) {
      return false;  // idling between cycles, matching simbridge's own ND frame validity duration
    }
    this->beginCycle(config);
  } else if (configChanged) {
    this->beginCycle(config);  // restart with fresh geometry rather than finish a now-stale cycle
  }

  if (this->_phase == Phase::Filling) {
    const int visibleRows = std::max(1, this->_visibleRowEnd - this->_visibleRowStart);
    const int rowsPerBatch = std::max(1, (visibleRows + kFillBatchCount - 1) / kFillBatchCount);
    const int rowStart = this->_fillRow;
    const int rowEnd = std::min(this->_visibleRowEnd, rowStart + rowsPerBatch);
    this->fillRows(terrain, rowStart, rowEnd);
    this->_fillRow = rowEnd;

    if (this->_fillRow >= this->_visibleRowEnd) {
      this->computeThresholds(terrain);
      this->_phase = Phase::Coloring;
    }
    return false;
  }

  // Phase::Coloring
  const int visiblePatchRows = std::max(1, this->_visiblePatchRowEnd - this->_visiblePatchRowStart);
  const int patchRowsPerBatch = std::max(1, (visiblePatchRows + kColorBatchCount - 1) / kColorBatchCount);
  const int patchRowStart = this->_colorPatchRow;
  const int patchRowEnd = std::min(this->_visiblePatchRowEnd, patchRowStart + patchRowsPerBatch);
  this->colorPatchRows(patchRowStart, patchRowEnd);
  this->_colorPatchRow = patchRowEnd;

  if (this->_colorPatchRow < this->_visiblePatchRowEnd) {
    return false;  // still mid-cycle
  }

  this->publishThresholds();
  this->_phase = Phase::Idle;
  this->_idleUntilSeconds = simTimeSeconds + kIdleSecondsAfterCycle;
  return true;
}

#endif  // A380X
