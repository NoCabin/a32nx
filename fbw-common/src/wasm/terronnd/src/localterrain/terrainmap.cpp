#include "terrainmap.h"

// This translation unit is only meaningful for the A380X (see fbw-common/src/wasm/terronnd/src/navigationdisplay,
// where it's used behind #ifdef A380X); guarding the whole implementation here too keeps it from linking any
// zlib/terrain-map code into the A32NX build, since build.sh's compile flags don't pass -ffunction-sections
// (so wasm-ld's --gc-sections can't strip unused *functions*, only unused data).
#ifdef A380X

#include <cmath>
#include <cstdio>
#include <cstring>

#include "zlib.h"

using namespace localterrain;

namespace {

std::uint16_t readU16LE(const std::uint8_t* p) {
  return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::int16_t readI16LE(const std::uint8_t* p) {
  return static_cast<std::int16_t>(readU16LE(p));
}

std::uint32_t readU32LE(const std::uint8_t* p) {
  return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
         (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

std::int8_t readI8(const std::uint8_t* p) {
  return static_cast<std::int8_t>(p[0]);
}

// Inflates a single gzip member (a tile's compressed payload) fully into a buffer that is already sized to
// exactly hold the decompressed output (rows * columns * sizeof(int16_t) -- known up front from the tile
// header), so this is always a single-shot inflate with no growable output handling needed.
bool gunzipExact(const std::uint8_t* src, std::size_t srcLen, std::uint8_t* dst, std::size_t dstLen) {
  z_stream strm;
  std::memset(&strm, 0, sizeof(strm));

  // windowBits = 15 + 16 requests gzip-header/trailer decoding (not raw deflate).
  if (inflateInit2(&strm, 15 + 16) != Z_OK) {
    return false;
  }

  strm.next_in = const_cast<Bytef*>(src);
  strm.avail_in = static_cast<uInt>(srcLen);
  strm.next_out = dst;
  strm.avail_out = static_cast<uInt>(dstLen);

  const int result = inflate(&strm, Z_FINISH);
  const bool ok = (result == Z_STREAM_END) && (strm.total_out == dstLen);
  inflateEnd(&strm);
  return ok;
}

}  // namespace

std::int64_t TerrainMap::gridCellKey(std::int32_t southwestLatitude, std::int32_t southwestLongitude) {
  // +1000 offsets keep both components positive (actual ranges are lat [-90,90], lon [-180,180]) so they pack
  // into one key with no collisions.
  return (static_cast<std::int64_t>(southwestLatitude) + 1000) * 100000 + (static_cast<std::int64_t>(southwestLongitude) + 1000);
}

bool TerrainMap::load(const char* path) {
  this->_loaded = false;
  this->_catalog.clear();
  this->_catalogIndexByCell.clear();
  this->_cache.clear();
  this->_cacheSlotByCatalogIndex.clear();

  FILE* file = std::fopen(path, "rb");
  if (!file) {
    return false;
  }

  std::uint8_t header[14];
  if (std::fread(header, 1, sizeof(header), file) != sizeof(header)) {
    std::fclose(file);
    return false;
  }
  this->_angularStepLatitude = header[8];
  this->_angularStepLongitude = header[9];

  std::uint8_t tileHeader[11];
  std::uint64_t offset = sizeof(header);
  while (true) {
    if (std::fseek(file, static_cast<long>(offset), SEEK_SET) != 0) {
      break;
    }
    const std::size_t read = std::fread(tileHeader, 1, sizeof(tileHeader), file);
    if (read != sizeof(tileHeader)) {
      break;
    }

    TileCatalogEntry entry;
    entry.rows = readU16LE(&tileHeader[0]);
    entry.columns = readU16LE(&tileHeader[2]);
    entry.southwestLatitude = readI8(&tileHeader[4]);
    entry.southwestLongitude = readI16LE(&tileHeader[5]);
    entry.compressedByteCount = readU32LE(&tileHeader[7]);
    entry.fileOffset = offset + sizeof(tileHeader);

    if (entry.rows == 0 || entry.columns == 0 || entry.compressedByteCount == 0) {
      // corrupted/unexpected entry -- stop rather than risk walking the rest of the file out of sync
      break;
    }

    this->_catalogIndexByCell[gridCellKey(entry.southwestLatitude, entry.southwestLongitude)] = this->_catalog.size();
    this->_catalog.push_back(entry);
    offset = entry.fileOffset + entry.compressedByteCount;
  }

  std::fclose(file);
  this->_loaded = !this->_catalog.empty();
  return this->_loaded;
}

const TerrainMap::CachedTile* TerrainMap::ensureTileCached(std::size_t catalogIndex) const {
  {
    const auto it = this->_cacheSlotByCatalogIndex.find(catalogIndex);
    if (it != this->_cacheSlotByCatalogIndex.end()) {
      CachedTile& cached = this->_cache[it->second];
      cached.lastUseTick = ++this->_tick;
      return &cached;
    }
  }

  const TileCatalogEntry& entry = this->_catalog[catalogIndex];

  // NOTE: keeping the file handle open across the object's lifetime would save repeated fopen/fclose calls,
  // but WASI's file-descriptor budget under the MSFS sandbox is limited and shared with the rest of the
  // module; opening only for the duration of a single tile read is the safer default here.
  FILE* file = std::fopen("\\work\\terrain\\terrain.map", "rb");
  if (!file) {
    return nullptr;
  }

  std::vector<std::uint8_t> compressed(entry.compressedByteCount);
  if (std::fseek(file, static_cast<long>(entry.fileOffset), SEEK_SET) != 0 ||
      std::fread(compressed.data(), 1, compressed.size(), file) != compressed.size()) {
    std::fclose(file);
    return nullptr;
  }
  std::fclose(file);

  const std::size_t elevationCount = static_cast<std::size_t>(entry.rows) * entry.columns;
  auto elevationFt = std::make_unique<std::int16_t[]>(elevationCount);
  if (!gunzipExact(compressed.data(), compressed.size(), reinterpret_cast<std::uint8_t*>(elevationFt.get()),
                    elevationCount * sizeof(std::int16_t))) {
    return nullptr;
  }

  // decompressed values are little-endian int16 metres (or -1 for water/void); convert to feet in place,
  // matching simbridge's Tile.loadElevationGrid() exactly (round(metres * 3.28084), -1 stays -1).
  for (std::size_t i = 0; i < elevationCount; ++i) {
    const std::int16_t raw = readI16LE(reinterpret_cast<const std::uint8_t*>(&elevationFt[i]));
    if (raw == -1) {
      elevationFt[i] = -1;
    } else {
      elevationFt[i] = static_cast<std::int16_t>(std::lround(static_cast<double>(raw) * 3.28084));
    }
  }

  if (this->_cache.size() >= kMaxCachedTiles) {
    // find the least-recently-used slot (only scans up to kMaxCachedTiles entries, and only on eviction --
    // not per sample, unlike the lookups above)
    std::size_t oldestIdx = 0;
    long oldestTick = this->_tick + 1;
    for (std::size_t i = 0; i < this->_cache.size(); ++i) {
      if (this->_cache[i].lastUseTick < oldestTick) {
        oldestTick = this->_cache[i].lastUseTick;
        oldestIdx = i;
      }
    }

    // swap-and-pop removal so no index in _cacheSlotByCatalogIndex needs shifting -- only the evicted slot and
    // whichever tile ends up moved into it change.
    this->_cacheSlotByCatalogIndex.erase(this->_cache[oldestIdx].catalogIndex);
    const std::size_t lastIdx = this->_cache.size() - 1;
    if (oldestIdx != lastIdx) {
      this->_cache[oldestIdx] = std::move(this->_cache[lastIdx]);
      this->_cacheSlotByCatalogIndex[this->_cache[oldestIdx].catalogIndex] = oldestIdx;
    }
    this->_cache.pop_back();
  }

  CachedTile tile;
  tile.catalogIndex = catalogIndex;
  tile.rows = entry.rows;
  tile.columns = entry.columns;
  tile.elevationFt = std::move(elevationFt);
  tile.lastUseTick = ++this->_tick;
  this->_cache.push_back(std::move(tile));
  this->_cacheSlotByCatalogIndex[catalogIndex] = this->_cache.size() - 1;
  return &this->_cache.back();
}

float TerrainMap::sampleElevationFt(double latitude, double longitude) const {
  if (!this->_loaded) {
    return UnknownElevation;
  }

  double lon = longitude;
  while (lon > 180.0) lon -= 360.0;
  while (lon < -180.0) lon += 360.0;

  // O(1) lookup of the covering tile's grid cell -- this used to scan the entire catalog (tens of thousands of
  // entries on the real terrain.map) on every single pixel sample; see _catalogIndexByCell's comment.
  const std::int32_t latCell =
      static_cast<std::int32_t>(std::floor(latitude / this->_angularStepLatitude)) * this->_angularStepLatitude;
  const std::int32_t lonCell =
      static_cast<std::int32_t>(std::floor(lon / this->_angularStepLongitude)) * this->_angularStepLongitude;

  const auto it = this->_catalogIndexByCell.find(gridCellKey(latCell, lonCell));
  if (it == this->_catalogIndexByCell.end()) {
    return WaterElevation;
  }

  const std::size_t catalogIndex = it->second;
  const TileCatalogEntry& entry = this->_catalog[catalogIndex];
  const double sw_lat = entry.southwestLatitude;
  const double sw_lon = entry.southwestLongitude;
  const double ne_lat = sw_lat + this->_angularStepLatitude;
  const double ne_lon = sw_lon + this->_angularStepLongitude;

  const CachedTile* tile = this->ensureTileCached(catalogIndex);
  if (!tile || tile->rows == 0 || tile->columns == 0) {
    return UnknownElevation;
  }

  // row 0 is the northernmost row (matches simbridge's ElevationGrid.worldToGridIndices convention)
  const double rowStepDeg = this->_angularStepLatitude / static_cast<double>(tile->rows);
  const double colStepDeg = this->_angularStepLongitude / static_cast<double>(tile->columns);

  double fRow = (ne_lat - latitude) / rowStepDeg;
  double fCol = (lon - sw_lon) / colStepDeg;

  int r0 = static_cast<int>(fRow);
  int c0 = static_cast<int>(fCol);
  if (r0 < 0) r0 = 0;
  if (r0 >= tile->rows) r0 = tile->rows - 1;
  if (c0 < 0) c0 = 0;
  if (c0 >= tile->columns) c0 = tile->columns - 1;
  int r1 = r0 + 1 >= tile->rows ? r0 : r0 + 1;
  int c1 = c0 + 1 >= tile->columns ? c0 : c0 + 1;

  double tr = fRow - static_cast<double>(static_cast<int>(fRow));
  double tc = fCol - static_cast<double>(static_cast<int>(fCol));
  if (tr < 0.0) tr = 0.0;
  if (tr > 1.0) tr = 1.0;
  if (tc < 0.0) tc = 0.0;
  if (tc > 1.0) tc = 1.0;

  const std::int16_t v00 = tile->elevationFt[static_cast<std::size_t>(r0) * tile->columns + static_cast<std::size_t>(c0)];
  const std::int16_t v01 = tile->elevationFt[static_cast<std::size_t>(r0) * tile->columns + static_cast<std::size_t>(c1)];
  const std::int16_t v10 = tile->elevationFt[static_cast<std::size_t>(r1) * tile->columns + static_cast<std::size_t>(c0)];
  const std::int16_t v11 = tile->elevationFt[static_cast<std::size_t>(r1) * tile->columns + static_cast<std::size_t>(c1)];

  // never interpolate across a water/void sample -- nearest-neighbour instead of blending a bogus average
  if (v00 == -1 || v01 == -1 || v10 == -1 || v11 == -1) {
    return WaterElevation;
  }

  const double top = v00 + (v01 - v00) * tc;
  const double bottom = v10 + (v11 - v10) * tc;
  return static_cast<float>(top + (bottom - top) * tr);
}

#endif  // A380X
