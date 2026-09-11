#pragma once

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-function"
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma clang diagnostic ignored "-Wsign-conversion"
#include <MSFS/Legacy/gauges.h>
#include <MSFS/Render/nanovg.h>
#include <MSFS/Render/stb_image.h>
#pragma clang diagnostic pop
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string_view>
#include <vector>

#define FMT_HEADER_ONLY
#include <fmt/format.h>

#include "../simconnect/clientdataarea.hpp"
#include "../simconnect/connection.hpp"
#include "../simconnect/lvarobject.hpp"
#include "../simconnect/simobject.hpp"
#include "../types/arinc429.hpp"
#include "../types/quantity.hpp"
#include "../types/simbridge.h"
#include "configuration.h"

#ifdef A380X
#include "../localterrain/ndrenderer.h"
#include "../localterrain/terrainmap.h"
#endif

namespace navigationdisplay {

#ifdef A380X
/**
 * @brief Live aircraft/EFIS state Collection gathers each frame, handed to DisplayBase::render() so it can
 * drive LocalNdRenderer when SimBridge has stopped sending frames. See DisplayBase::render() for the
 * SimBridge-connectivity watchdog itself.
 */
struct LocalRenderInputs {
  double simTimeSeconds = 0.0;

  bool aircraftPositionValid = false;
  double aircraftLatitude = 0.0;
  double aircraftLongitude = 0.0;
  double headingDeg = 0.0;
  double altitudeFt = 0.0;
  double verticalSpeedFtMin = 0.0;
  bool gearIsDown = false;

  bool destinationValid = false;
  double destinationLatitude = 0.0;
  double destinationLongitude = 0.0;

  // shared across both displays -- loaded lazily on first use, see Collection
  std::shared_ptr<localterrain::TerrainMap> terrainMap;
};
#endif

/**
 * @brief Defines the different display sides
 */
enum DisplaySide { Left = 'L', Right = 'R' };

/**
 * @brief The base class for a display
 */
class DisplayBase {
 public:
  struct NdConfiguration {
    types::Length range;
    std::uint8_t mode;
    bool terrOnNd;
    bool terrOnVd;
    float potentiometer;
    bool powered;
  };

  DisplayBase(const DisplayBase&) = delete;
  virtual ~DisplayBase() { this->destroy(); }

  DisplayBase& operator=(const DisplayBase&) = delete;

  virtual void update(const NdConfiguration& config) = 0;

  DisplaySide side() const;
  void destroy();
#ifdef A380X
  void render(sGaugeDrawData* pDrawData, const LocalRenderInputs& localInputs);
#else
  void render(sGaugeDrawData* pDrawData);
#endif

#ifdef A380X
  // Implemented by Display<> below, where the NdMinElevation/... LVar template parameters live.
  virtual void writeLocalThresholds(const localterrain::NdFrameResult& frame) = 0;
#endif

 protected:
  DisplaySide _side;
  NdConfiguration _configuration;
  std::size_t _frameBufferSize;
  int _nanovgImage;
  NVGcontext* _context;
  std::shared_ptr<simconnect::ClientDataArea<types::ThresholdData>> _thresholds;
  std::shared_ptr<simconnect::ClientDataAreaBuffered<std::uint8_t, SIMCONNECT_CLIENTDATA_MAX_SIZE>> _frameData;

#ifdef A380X
  // The frameData/thresholds SimConnect callbacks (Display<>'s constructor below) just set this flag --
  // they run during connection.readData(), before render() runs in the same PRE_DRAW tick, so render() can
  // safely consume-and-clear it once per frame rather than needing a raw timestamp inside the callback.
  bool _simBridgeSignalPending = false;
  bool _everSawSimBridgeSignal = false;
  double _secondsSinceLastSimBridgeSignal = 0.0;
  bool _localModeActive = false;
  localterrain::LocalNdRenderer _localRenderer;

  static constexpr double kSimBridgeTimeoutSeconds = 3.0;

  void updateSimBridgeWatchdog(double dt);
  void renderLocal(sGaugeDrawData* pDrawData, const LocalRenderInputs& localInputs);
#endif

  DisplayBase(DisplaySide side, FsContext context);

  void destroyImage();
  void drawCurrentImage(sGaugeDrawData* pDrawData);

  static constexpr std::size_t MaxFrameByteCount = 4 * 1024 * 1024;
  static constexpr std::uint32_t MaxFrameDimension = 4096;
};

/**
 * @brief Template class with functionality to manage the terrain on ND data
 * @tparam NdMinElevation Aircraft variable name to define the minimum elevation
 * @tparam NdMinElevationMode Aircraft variable name to define the minimum elevation mode
 * @tparam NdMaxElevation Aircraft variable name to define the maximum elevation
 * @tparam NdMaxElevationMode Aircraft variable name to define the maximum elevation mode
 */
template <std::string_view const& NdMinElevation,
          std::string_view const& NdMinElevationMode,
          std::string_view const& NdMaxElevation,
          std::string_view const& NdMaxElevationMode>
class Display : public DisplayBase {
 private:
  std::shared_ptr<simconnect::LVarObject<NdMinElevation, NdMinElevationMode, NdMaxElevation, NdMaxElevationMode>> _ndThresholdData;
  bool _ignoreNextFrame;

  void resetNavigationDisplayData() {
    this->_ndThresholdData->template value<NdMinElevation>() = -1;
    this->_ndThresholdData->template value<NdMinElevationMode>() = 0;
    this->_ndThresholdData->template value<NdMaxElevation>() = -1;
    this->_ndThresholdData->template value<NdMaxElevationMode>() = 0;
    this->_ndThresholdData->writeValues();
  }

 public:
  /**
   * @brief Construct a new Display object
   *
   * Communcation concept to the SimBridge:
   *  - The threshold data block from the SimBridge contains the number of bytes for a frame
   *  - The framedata is sent afterwards in chunks of SIMCONNECT_CLIENTDATA_MAX_SIZE bytes per chunk, until the frame is transmitted
   *
   * @param connection The connection to SimCommect
   * @param side The display side
   * @param context The gauge context
   */
  Display(simconnect::Connection& connection, DisplaySide side, FsContext context)
      : DisplayBase(side, context), _ndThresholdData(nullptr), _ignoreNextFrame(false) {
    this->_ndThresholdData = connection.lvarObject<NdMinElevation, NdMinElevationMode, NdMaxElevation, NdMaxElevationMode>();

    // write initial values to avoid invalid drawings
    this->resetNavigationDisplayData();

    this->_frameData = connection.clientDataArea<std::uint8_t, SIMCONNECT_CLIENTDATA_MAX_SIZE>();
    this->_frameData->defineArea(side == DisplaySide::Left ? FrameDataLeftName : FrameDataRightName);
    this->_frameData->requestArea(SIMCONNECT_CLIENT_DATA_PERIOD_ON_SET);
    this->_frameData->setOnChangeCallback([=]() {
#ifdef A380X
      this->_simBridgeSignalPending = true;
      this->_localModeActive = false;
      this->_localRenderer.reset();
#endif
      if (!this->_ignoreNextFrame && (this->_configuration.terrOnNd || this->_configuration.terrOnVd)) {
        if (this->_nanovgImage == 0) {
          // If we don't have an image yet, create one
          this->_nanovgImage =
              nvgCreateImageMem(this->_context, 0, this->_frameData->data().data(), static_cast<int>(this->_frameBufferSize));
          if (this->_nanovgImage == 0) {
            const char* reason = stbi_failure_reason();
            std::cerr << fmt::format("TERR ON ND: Unable to create the image from the stream. Reason: {}",
                                     reason != nullptr ? reason : "unknown")
                      << std::endl;
          }

          return;
        }

        // Otherwise, decode the PNG manually and update the existing image
        int decodedWidth, decodedHeight;
        uint8_t* decodedImage = stbi_load_from_memory(this->_frameData->data().data(), static_cast<int>(this->_frameBufferSize),
                                                      &decodedWidth, &decodedHeight, nullptr, 4);
        if (decodedImage == nullptr) {
          const char* reason = stbi_failure_reason();
          std::cerr << fmt::format("TERR ON ND: Unable to create the image from the stream. Reason: {}",
                                   reason != nullptr ? reason : "unknown")
                    << std::endl;
          return;
        }

        int width, height;
        nvgImageSize(this->_context, this->_nanovgImage, &width, &height);

        if (decodedWidth != width || decodedHeight != height) {
          // This should never happen, but bail just in case
          std::cerr << fmt::format("TERR ON ND: The image size does not match the expected size. Expected: {}x{}, actual: {}x{}", width,
                                   height, decodedWidth, decodedHeight);
          stbi_image_free(decodedImage);
          return;
        }

        nvgUpdateImage(this->_context, this->_nanovgImage, decodedImage);

        stbi_image_free(decodedImage);
      } else {
        this->resetNavigationDisplayData();
      }
    });

    this->_thresholds = connection.clientDataArea<types::ThresholdData>();
    this->_thresholds->defineArea(side == DisplaySide::Left ? ThresholdsLeftName : ThresholdsRightName);
    this->_thresholds->requestArea(SIMCONNECT_CLIENT_DATA_PERIOD_ON_SET);
    this->_thresholds->setAlwaysChanges(true);
    this->_thresholds->setOnChangeCallback([=]() {
#ifdef A380X
      this->_simBridgeSignalPending = true;
      this->_localModeActive = false;
      this->_localRenderer.reset();
#endif
      const std::uint32_t frameByteCount = this->_thresholds->data().frameByteCount;
      if (frameByteCount == 0 || frameByteCount > DisplayBase::MaxFrameByteCount) {
        // corrupted or incompatible packet: allocating this size could kill the module
        std::cerr << "TERR ON ND: Ignoring thresholds packet with implausible frame size: " << frameByteCount << std::endl;
        this->_frameBufferSize = 0;
        this->_frameData->reserve(0);
        return;
      }

      this->_frameBufferSize = frameByteCount;
      this->_frameData->reserve(this->_frameBufferSize);
      this->_ignoreNextFrame =
          this->_ignoreNextFrame &&
          (this->_thresholds->data().firstFrame == 0 || this->_configuration.mode != this->_thresholds->data().displayMode ||
           this->_configuration.range != (this->_thresholds->data().displayRange * types::nauticmile));

      if (!this->_ignoreNextFrame) {
        this->_ndThresholdData->template value<NdMinElevation>() = this->_thresholds->data().lowerThreshold;
        this->_ndThresholdData->template value<NdMinElevationMode>() = this->_thresholds->data().lowerThresholdMode;
        this->_ndThresholdData->template value<NdMaxElevation>() = this->_thresholds->data().upperThreshold;
        this->_ndThresholdData->template value<NdMaxElevationMode>() = this->_thresholds->data().upperThresholdMode;
        this->_ndThresholdData->writeValues();
      }
    });
  }
  Display(const Display&) = delete;
  virtual ~Display() {}

  Display& operator=(const Display&) = delete;

  /**
   * @brief Updates the configuration of the display and maybe resets the ND image
   * @param config The new ND configuration instance
   */
  void update(const DisplayBase::NdConfiguration& config) override {
    const bool resetMapData = this->_configuration.mode != config.mode || config.range != this->_configuration.range ||
                              this->_configuration.terrOnNd != config.terrOnNd || this->_configuration.terrOnVd != config.terrOnVd;
    const bool validEfisMode = config.mode == NavigationDisplayArcModeId || config.mode == NavigationDisplayRoseLsModeId ||
                               config.mode == NavigationDisplayRoseNavModeId || config.mode == NavigationDisplayRoseVorModeId;

    this->_configuration = config;
    this->_configuration.terrOnNd &= validEfisMode;

    if (!(this->_configuration.terrOnNd || this->_configuration.terrOnVd) || !validEfisMode || resetMapData) {
      this->resetNavigationDisplayData();
      this->destroyImage();
      this->_ignoreNextFrame = true;
    }
  }

#ifdef A380X
  void writeLocalThresholds(const localterrain::NdFrameResult& frame) override {
    this->_ndThresholdData->template value<NdMinElevation>() = static_cast<std::int16_t>(frame.minimumElevationFt);
    this->_ndThresholdData->template value<NdMinElevationMode>() = static_cast<std::uint8_t>(frame.minimumElevationMode);
    this->_ndThresholdData->template value<NdMaxElevation>() = static_cast<std::int16_t>(frame.maximumElevationFt);
    this->_ndThresholdData->template value<NdMaxElevationMode>() = static_cast<std::uint8_t>(frame.maximumElevationMode);
    this->_ndThresholdData->writeValues();
  }
#endif
};

/**
 * @brief Specialization of the display for the left side
 */
class DisplayLeft : public Display<NdLeftMinElevation, NdLeftMinElevationMode, NdLeftMaxElevation, NdLeftMaxElevationMode> {
 public:
  DisplayLeft(simconnect::Connection& connection, FsContext context)
      : Display<NdLeftMinElevation, NdLeftMinElevationMode, NdLeftMaxElevation, NdLeftMaxElevationMode>(connection,
                                                                                                        DisplaySide::Left,
                                                                                                        context) {}
  DisplayLeft(const DisplayLeft&) = delete;
  virtual ~DisplayLeft() {}

  DisplayLeft& operator=(const DisplayLeft&) = delete;
};

/**
 * @brief Specialization of the display for the right side
 */
class DisplayRight : public Display<NdRightMinElevation, NdRightMinElevationMode, NdRightMaxElevation, NdRightMaxElevationMode> {
 public:
  DisplayRight(simconnect::Connection& connection, FsContext context)
      : Display<NdRightMinElevation, NdRightMinElevationMode, NdRightMaxElevation, NdRightMaxElevationMode>(connection,
                                                                                                            DisplaySide::Right,
                                                                                                            context) {}
  DisplayRight(const DisplayRight&) = delete;
  virtual ~DisplayRight() {}

  DisplayRight& operator=(const DisplayRight&) = delete;
};

}  // namespace navigationdisplay
