#pragma once

#include <cstdint>
#include <vector>

#include "terrainmap.h"

namespace localterrain {

// Mirrors types::ThresholdMode (types/simbridge.h) 1:1 -- deliberately not reusing that type here, since it
// would pull in <SimConnect.h> (via simbridge.h) for a plain 3-value enum, and simbridge.h is only safe to
// include after MSFS/Legacy/gauges.h has already defined the Windows typedefs (DWORD etc.) SimConnect.h needs;
// this header has no reason to depend on that include order. displaybase.cpp casts this to
// types::ThresholdMode's underlying values when writing the LVars (see DisplayBase::writeLocalThresholds).
enum class ThresholdMode : std::uint8_t { PEAKS_MODE = 0, WARNING = 1, CAUTION = 2 };

struct NdFrameConfig {
  double aircraftLatitude;
  double aircraftLongitude;
  double headingDeg;
  double altitudeFt;
  double verticalSpeedFtMin;
  bool gearIsDown;

  bool destinationValid;
  double destinationLatitude;
  double destinationLongitude;

  double rangeNm;
  bool arcMode;
  // Rendered at the WASM gauge's own actual reported canvas size (sGaugeDrawData::winWidth/winHeight) rather
  // than an assumed fixed buffer size -- see the file comment below for why.
  int mapWidth;
  int mapHeight;
};

struct NdFrameResult {
  int width = 0;
  int height = 0;
  std::vector<std::uint8_t> rgba;  // width * height * 4, RGBA8

  float minimumElevationFt = -1.0f;
  ThresholdMode minimumElevationMode = ThresholdMode::PEAKS_MODE;
  float maximumElevationFt = -1.0f;
  ThresholdMode maximumElevationMode = ThresholdMode::PEAKS_MODE;
};

/**
 * @brief Locally renders an A380X-style terrain-on-ND frame straight from a TerrainMap, using the same
 * threshold/colour rules as flybywiresim SimBridge (see colorlogic.h), without needing SimBridge running.
 *
 * SimBridge's own kernel clips the display to the ND's actual arc/rose shape via a baked pixel pattern
 * texture (value 0 == outside the shape == fully transparent, see colorlogic.h's file header for why that
 * texture isn't reproduced here). isPixelVisible() (ndrenderer.cpp) is a geometric stand-in for that same clip,
 * derived from the ND's own SVG source (fbw-common/src/systems/instruments/src/ND/pages/{arc,rose}/*Underlay.tsx):
 * the outer range ring is a circle of radius 492 (ARC) / 250 (ROSE) in the ND's 768-wide SVG viewBox, aircraft
 * symbol always horizontally centered and exactly `radius` pixels down from the top of the visible circle.
 * Rather than assume this WASM's own canvas is some fixed pixel size matching that 768-wide reference (which
 * can't be verified from here), NdFrameConfig::mapWidth/mapHeight are set to the gauge's own actual reported
 * sGaugeDrawData::winWidth/winHeight each cycle, and the ring geometry is scaled proportionally from mapWidth
 * (492.0/768.0 etc) -- correct regardless of what the real canvas size turns out to be.
 *
 * Both the elevation sampling (one geodesic projection + tile lookup per visible pixel) and the histogram/
 * colour pass are spread across several update() calls in row batches -- mirroring the technique used by the
 * Horizon-787's own local terrain WASM renderer -- rather than done synchronously in a single frame, so a
 * refresh cycle doesn't spike the main thread.
 */
class LocalNdRenderer {
 public:
  /**
   * @brief Advances the (possibly multi-frame) render cycle. Returns true once a new completed frame is
   * available in result() -- callers should only push a new texture to the GPU when this returns true.
   */
  bool update(const NdFrameConfig& config, const TerrainMap& terrain, double simTimeSeconds);

  const NdFrameResult& result() const { return this->_result; }

  void reset();

 private:
  enum class Phase { Idle, Filling, Coloring };

  static constexpr int kFillBatchCount = 16;
  static constexpr int kColorBatchCount = 16;
  static constexpr double kIdleSecondsAfterCycle = 2.5;  // matches simbridge's own ND frame validity duration

  Phase _phase = Phase::Idle;
  int _fillRow = 0;
  int _colorPatchRow = 0;
  double _idleUntilSeconds = 0.0;

  // the row range that can possibly contain a visible pixel (see visibleRowRange() in ndrenderer.cpp) --
  // batching iterates only this range instead of [0, mapHeight), which matters when the gauge's actual canvas
  // is considerably taller than the visible ring (e.g. everything below the aircraft in ARC mode).
  int _visibleRowStart = 0;
  int _visibleRowEnd = 0;
  int _visiblePatchRowStart = 0;
  int _visiblePatchRowEnd = 0;

  NdFrameConfig _cycleConfig{};
  std::vector<float> _elevationGrid;

  // computed once, right after fill completes, then held for every batch of the color phase
  double _referenceAltitude = 0.0;
  double _minElevation = -1.0;
  double _maxElevation = 0.0;
  double _flatEarth = 0.0;
  double _halfElevation = 0.0;
  double _gearDownAltitudeOffset = 0.0;
  double _cutOffAltitude = 0.0;
  double _lowerPercentileElevation = 0.0;
  double _upperPercentileElevation = 0.0;
  bool _normalMode = true;

  NdFrameResult _result;

  void beginCycle(const NdFrameConfig& config);
  void fillRows(const TerrainMap& terrain, int rowStart, int rowEnd);
  void computeThresholds(const TerrainMap& terrain);
  void colorPatchRows(int patchRowStart, int patchRowEnd);
  void publishThresholds();
};

}  // namespace localterrain
