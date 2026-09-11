#include "display.h"

#ifdef A380X
#define INSTRUMENT_BG_COLOR nvgRGBA(0, 0, 0, 255)
#endif

#ifndef A380X
#define INSTRUMENT_BG_COLOR nvgRGBA(4, 4, 4, 255)
#endif

using namespace navigationdisplay;

DisplayBase::DisplayBase(DisplaySide side, FsContext context)
    : _side(side), _configuration(), _frameBufferSize(0), _nanovgImage(0), _context(nullptr), _thresholds(nullptr), _frameData(nullptr) {
  NVGparams params;
  params.userPtr       = context;
  params.edgeAntiAlias = false;
  this->_context       = nvgCreateInternal(&params);
}

DisplaySide DisplayBase::side() const {
  return this->_side;
}

void DisplayBase::destroy() {
  this->destroyImage();
  nvgDeleteInternal(this->_context);
  this->_context = nullptr;
}

void DisplayBase::destroyImage() {
  if (this->_nanovgImage != 0) {
    nvgDeleteImage(this->_context, this->_nanovgImage);
    this->_nanovgImage = 0;
  }
}

void DisplayBase::drawCurrentImage(sGaugeDrawData* pDrawData) {
  const float ratio = static_cast<float>(pDrawData->fbWidth) / static_cast<float>(pDrawData->fbHeight);
  nvgBeginFrame(this->_context, static_cast<float>(pDrawData->winWidth), static_cast<float>(pDrawData->winHeight), ratio);
  {
    if (this->_configuration.powered) {
      nvgFillColor(this->_context, INSTRUMENT_BG_COLOR);
      nvgBeginPath(this->_context);
      nvgRect(this->_context, 0.0f, 0.0f, static_cast<float>(pDrawData->winWidth), static_cast<float>(pDrawData->winHeight));
      nvgFill(this->_context);

      if (this->_nanovgImage != 0 && !helper::Math::almostEqual(this->_configuration.potentiometer, 0.0f)) {
        // draw the image
        nvgBeginPath(this->_context);
        NVGpaint imagePaint =
            nvgImagePattern(this->_context, 0.0f, 0.0f, static_cast<float>(pDrawData->winWidth), static_cast<float>(pDrawData->winHeight),
                            0.0, this->_nanovgImage, this->_configuration.potentiometer);
        nvgRect(this->_context, 0.0f, 0.0f, static_cast<float>(pDrawData->winWidth), static_cast<float>(pDrawData->winHeight));
        nvgFillPaint(this->_context, imagePaint);
        nvgFill(this->_context);
      }
    } else {
      nvgFillColor(this->_context, nvgRGBA(0, 0, 0, 255));
      nvgBeginPath(this->_context);
      nvgRect(this->_context, 0.0f, 0.0f, static_cast<float>(pDrawData->winWidth), static_cast<float>(pDrawData->winHeight));
      nvgFill(this->_context);
    }
  }
  nvgEndFrame(this->_context);
}

#ifdef A380X

void DisplayBase::updateSimBridgeWatchdog(double dt) {
  if (this->_simBridgeSignalPending) {
    this->_everSawSimBridgeSignal          = true;
    this->_secondsSinceLastSimBridgeSignal = 0.0;
    this->_simBridgeSignalPending          = false;
  } else if (this->_everSawSimBridgeSignal) {
    this->_secondsSinceLastSimBridgeSignal += dt;
  }
}

void DisplayBase::renderLocal(sGaugeDrawData* pDrawData, const LocalRenderInputs& localInputs) {
  if (!localInputs.aircraftPositionValid || localInputs.terrainMap == nullptr || !localInputs.terrainMap->loaded()) {
    // nothing sane to render locally -- leave whatever is currently displayed (matches the pre-existing
    // "no data yet" behaviour elsewhere in this file rather than flashing something bogus)
    return;
  }
  if (pDrawData->winWidth <= 0 || pDrawData->winHeight <= 0) {
    return;
  }

  const bool arcMode = this->_configuration.mode == NavigationDisplayArcModeId;

  localterrain::NdFrameConfig config;
  config.aircraftLatitude    = localInputs.aircraftLatitude;
  config.aircraftLongitude   = localInputs.aircraftLongitude;
  config.headingDeg          = localInputs.headingDeg;
  config.altitudeFt          = localInputs.altitudeFt;
  config.verticalSpeedFtMin  = localInputs.verticalSpeedFtMin;
  config.gearIsDown          = localInputs.gearIsDown;
  config.destinationValid    = localInputs.destinationValid;
  config.destinationLatitude = localInputs.destinationLatitude;
  config.destinationLongitude = localInputs.destinationLongitude;
  config.rangeNm              = static_cast<double>(this->_configuration.range.convert(types::nauticmile));
  config.arcMode               = arcMode;
  // Render at the gauge's own actual canvas size rather than an assumed fixed buffer -- see ndrenderer.h's
  // file comment for why.
  config.mapWidth  = pDrawData->winWidth;
  config.mapHeight = pDrawData->winHeight;

  const bool haveNewFrame = this->_localRenderer.update(config, *localInputs.terrainMap, localInputs.simTimeSeconds);
  if (!haveNewFrame) {
    return;
  }

  const localterrain::NdFrameResult& frame = this->_localRenderer.result();
  if (frame.width <= 0 || frame.height <= 0 || frame.rgba.empty()) {
    return;
  }

  if (this->_nanovgImage == 0) {
    this->_nanovgImage = nvgCreateImageRGBA(this->_context, frame.width, frame.height, 0, frame.rgba.data());
  } else {
    int width, height;
    nvgImageSize(this->_context, this->_nanovgImage, &width, &height);
    if (width == frame.width && height == frame.height) {
      nvgUpdateImage(this->_context, this->_nanovgImage, frame.rgba.data());
    } else {
      this->destroyImage();
      this->_nanovgImage = nvgCreateImageRGBA(this->_context, frame.width, frame.height, 0, frame.rgba.data());
    }
  }

  this->writeLocalThresholds(frame);
}

void DisplayBase::render(sGaugeDrawData* pDrawData, const LocalRenderInputs& localInputs) {
  if (this->_context == nullptr) {
    return;
  }

  this->updateSimBridgeWatchdog(pDrawData->dt);

  const bool terrainWanted = this->_configuration.powered && this->_configuration.terrOnNd &&
                             !helper::Math::almostEqual(this->_configuration.potentiometer, 0.0f);
  const bool simBridgeDown = !this->_everSawSimBridgeSignal || this->_secondsSinceLastSimBridgeSignal >= kSimBridgeTimeoutSeconds;
  this->_localModeActive   = terrainWanted && simBridgeDown;

  if (this->_localModeActive) {
    this->renderLocal(pDrawData, localInputs);
  }

  this->drawCurrentImage(pDrawData);
}

#else

void DisplayBase::render(sGaugeDrawData* pDrawData) {
  if (this->_context == nullptr) {
    return;
  }

  this->drawCurrentImage(pDrawData);
}

#endif
