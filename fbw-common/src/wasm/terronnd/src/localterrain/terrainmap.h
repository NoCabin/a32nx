#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

namespace localterrain {

// Sentinels shared 1:1 with simbridge's apps/server/src/terrain/processing/generic/constants.ts,
// so the local renderer's color logic (ported from the same source) can use the same checks.
static constexpr float WaterElevation = -1.0f;
static constexpr float UnknownElevation = 32766.0f;
static constexpr float InvalidElevation = 32767.0f;

/**
 * @brief Reads and samples flybywiresim SimBridge's own terrain.map file (apps/server/src/terrain/fileformat
 * in the simbridge repo), so a locally-rendered frame uses the exact same elevation data SimBridge would have
 * used, without requiring SimBridge itself to be running.
 *
 * The file is a flat sequence of gzip-compressed tiles behind a 14-byte header; this class scans that sequence
 * once at load time to build an in-memory catalog (tile bounds + file offset/size only, no decompression), then
 * lazily gzip-inflates individual tiles into a small bounded LRU cache as they're sampled. Shared by both ND
 * displays (see Collection) so the catalog and cache are only held once.
 */
class TerrainMap {
 public:
  TerrainMap() = default;
  TerrainMap(const TerrainMap&) = delete;
  TerrainMap& operator=(const TerrainMap&) = delete;

  /**
   * @brief Scans the terrain.map file's tile catalog. Safe to call once; a missing/unreadable file just means
   * sampleElevationFt() returns UnknownElevation everywhere, same convention as an uncovered area.
   * @param path Package-relative path, e.g. "\\work\\terrain\\terrain.map"
   */
  bool load(const char* path);

  bool loaded() const { return this->_loaded; }

  float sampleElevationFt(double latitude, double longitude) const;

 private:
  struct TileCatalogEntry {
    std::int32_t southwestLatitude;
    std::int32_t southwestLongitude;
    std::uint16_t rows;
    std::uint16_t columns;
    std::uint64_t fileOffset;
    std::uint32_t compressedByteCount;
  };

  struct CachedTile {
    std::size_t catalogIndex;
    std::uint16_t rows;
    std::uint16_t columns;
    std::unique_ptr<std::int16_t[]> elevationFt;
    long lastUseTick;
  };

  // terrain.map's tiles are only 1deg x 1deg (~278x278 cells, ~154KB decompressed each) -- a single ND sweep
  // at long range can touch on the order of a hundred distinct tiles, so a small cache thrashes (re-decompresses
  // the same tiles every cycle). 512 tiles is still only ~80MB resident, nowhere near SimBridge's own footprint.
  static constexpr std::size_t kMaxCachedTiles = 512;

  bool _loaded = false;
  std::uint8_t _angularStepLatitude = 0;
  std::uint8_t _angularStepLongitude = 0;
  std::vector<TileCatalogEntry> _catalog;

  // Direct O(1) lookup from a tile's (southwest latitude, southwest longitude) integer grid cell to its index
  // in _catalog -- built once in load(). sampleElevationFt() used to find the covering tile by scanning the
  // whole catalog (up to ~23k entries on the real terrain.map) on every single pixel; this replaces that scan.
  std::unordered_map<std::int64_t, std::size_t> _catalogIndexByCell;

  mutable std::vector<CachedTile> _cache;
  // catalogIndex -> position in _cache, kept in sync with swap-and-pop eviction in ensureTileCached() so tile
  // cache lookups are also O(1) instead of scanning up to kMaxCachedTiles entries per sample.
  mutable std::unordered_map<std::size_t, std::size_t> _cacheSlotByCatalogIndex;
  mutable long _tick = 0;

  static std::int64_t gridCellKey(std::int32_t southwestLatitude, std::int32_t southwestLongitude);
  const CachedTile* ensureTileCached(std::size_t catalogIndex) const;
};

}  // namespace localterrain
