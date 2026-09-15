// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

#ifndef FLYBYWIRE_AIRCRAFT_THRUSTLIMITS_GP7000_HPP
#define FLYBYWIRE_AIRCRAFT_THRUSTLIMITS_GP7000_HPP

#include <algorithm>
#include <map>
#include <sstream>

#include "logging.h"

#include "EngineRatios.hpp"
#include "Fadec.h"

// GP7270 thrust limits, derived from the real Airbus A380 FCOM (KAL A380 FLEET)
class ThrustLimits_GP7000 {
  static constexpr double limits[43][6] = {
  // TO
      {-2000,  35.000,  55.000,  97.800,  97.700,  74.400}, // row 0
      {0,      35.000,  50.000,  97.600,  97.600,  72.900}, //
      {2000,   25.000,  50.000,  97.400,  97.300,  97.300}, //
      {4000,   25.000,  45.000,  97.200,  97.000,  97.000}, //
      {6000,   25.000,  40.000,  97.000,  96.800,  96.800}, //
      {8000,   10.000,  35.000,  96.700,  96.200,  96.200}, //
      {10000,  5.000,   35.000,  97.700,  96.000,  96.000}, //
      {12000,  0.000,   30.000,  98.100,  95.700,  95.700}, //
      {14000, -5.000,   25.000,  98.100,  95.200,  95.200}, // row 8
  // GA
      {-2000,  45.000,  60.000,  98.200,  98.200,  0.000 }, // row 9
      {0,      25.000,  55.000,  98.200,  98.100,  0.000 }, //
      {2000,   30.000,  55.000,  97.900,  97.900,  0.000 }, //
      {4000,   25.000,  50.000,  97.900,  97.700,  0.000 }, //
      {6000,   20.000,  45.000,  97.800,  97.400,  0.000 }, //
      {8000,   20.000,  40.000,  97.500,  96.800,  0.000 }, //
      {10000,  10.000,  35.000,  97.500,  96.700,  0.000 }, //
      {12000,  0.000,   35.000,  98.200,  96.500,  0.000 }, //
      {14000,  0.000,   30.000,  98.300,  96.100,  0.000 }, //
      {16000, -5.000,   25.000,  99.000,  95.800,  0.000 }, // row 18
  // CLB
      {-1000,  35.000,  60.000,  98.400,  89.600,  0.000 }, // row 19
      {3000,   25.000,  55.000,  98.200,  91.100,  0.000 }, //
      {7000,   20.000,  50.000,  98.100,  93.100,  0.000 }, //
      {11000,  20.000,  50.000,  98.100,  98.000,  0.000 }, //
      {15000,  15.000,  45.000,  98.000,  97.700,  0.000 }, //
      {19000,  10.000,  40.000,  97.200,  96.600,  0.000 }, //
      {23000,  5.000,   35.000,  98.200,  96.800,  0.000 }, //
      {27000,  0.000,   35.000,  98.200,  96.700,  0.000 }, //
      {31000, -5.000,   30.000,  98.500,  96.800,  0.000 }, //
      {35000, -10.000,  20.000,  98.500,  96.700,  0.000 }, //
      {39000, -10.000,  15.000,  98.300,  96.500,  0.000 }, //
      {43000, -10.000,  15.000,  98.100,  95.900,  0.000 }, // row 30
  // MCT (230 KT)
      {-1000,  30.000,  60.000,  98.300,  89.500,  0.000 }, // row 31
      {3000,   25.000,  55.000,  98.000,  90.900,  0.000 }, //
      {7000,   20.000,  50.000,  98.100,  93.200,  0.000 }, //
      {11000,  10.000,  40.000,  97.000,  94.600,  0.000 }, //
      {15000,  5.000,   35.000,  98.200,  95.100,  0.000 }, //
      {19000,  0.000,   30.000,  98.400,  96.800,  0.000 }, //
      {23000, -5.000,   25.000,  98.500,  96.900,  0.000 }, //
      {27000, -10.000,  20.000,  96.400,  96.900,  0.000 }, //
      {31000, -15.000,  10.000,  98.700,  97.900,  0.000 }, //
      {35000, -10.000,  10.000,  98.500,  97.400,  0.000 }, //
      {39000, -10.000,  10.000,  98.300,  97.000,  0.000 }, //
      {43000, -20.000,  15.000,  97.800,  95.500,  0.000 }  // row 42
  };

 public:
  static int finder(double altitude, int index) {
    while (altitude >= limits[index][0]) {
      index++;
    }
    return index;
  }

  // Bleed corrections from the GP7270 FCOM power management tables (THR CORRECTIONS FOR AIR BLEED),
  // simplified to a single OAT<CP / OAT>CP split (the FCOM does not split further by altitude for
  // this engine). packsDelta/naiDelta apply directly; wai contributes naiDelta + waiAddDelta.
  static double bleedTotal(int    type,      //
                           double altitude,  //
                           double oat,       //
                           double cp,        //
                           double lp,        //
                           double flexTemp,  //
                           int    packs,     //
                           int    nacelle,   //
                           int    wing       //
  ) {
    if (flexTemp > lp && type <= 1) {
      return packs * -0.6 + nacelle * -0.7 + wing * -0.7;
    }

    const bool belowCp = oat < cp;

    double packsDelta = 0;
    double naiDelta    = 0;
    double waiAddDelta = 0;

    switch (type) {
      case 0:  // TO
        naiDelta    = belowCp ? 0.0 : -1.7;
        waiAddDelta = 0.0;
        break;
      case 1:  // GA
        naiDelta    = belowCp ? 0.0 : -1.9;
        waiAddDelta = belowCp ? 0.0 : -1.3;
        break;
      case 2:  // CLB
        packsDelta  = 0.4;
        naiDelta    = belowCp ? 0.0 : -3.3;
        waiAddDelta = belowCp ? 0.0 : -3.4;
        break;
      case 3:  // MCT
        packsDelta  = -1.2;
        naiDelta    = belowCp ? 0.0 : -3.1;
        waiAddDelta = belowCp ? 0.0 : -2.7;
        break;
    }

    return packs * packsDelta + nacelle * naiDelta + wing * (naiDelta + waiAddDelta);
  }

  // DCL thr% lookup tables (derated climb thrust as % of MCL N1), transcribed from the GP7270 FCOM
  // (PER-THR-NRM-MCL, PER-THR-NRM-DCL). Rows: TAT (C), Columns: altitude (ft). 100.0 fills cells
  // outside the chart's published envelope (matching the base A380X table's own convention).
  static constexpr double dclTatBreakpoints[19] = {
      -60, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60};

  static constexpr double dclAltBreakpoints[12] = {
      -1000, 3000, 7000, 11000, 15000, 19000, 23000, 27000, 31000, 35000, 39000, 43000};

  // [level][tatRow][altCol], level 0 = MAX CLB, 1 = DERATE 01, 2 = DERATE 02, 3 = DERATE 03
  static constexpr double dclThr[4][19][12] = {{
      // MAX CLB
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.5, 98.5, 98.3, 98.1},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.5, 98.5, 98.3, 98.1},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.5, 98.5, 98.3, 98.1},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.5, 98.5, 98.3, 98.1},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.5, 98.5, 98.3, 98.1},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.5, 98.4, 98.1, 97.6},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.2, 98.2, 97.9, 97.6, 97.2},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 98.2, 98.1, 98.0, 97.7, 97.3, 97.1},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.2, 97.9, 98.0, 98.0, 97.7, 97.3, 96.5},
      {98.4, 98.2, 98.1, 98.1, 98.0, 97.5, 97.7, 97.6, 97.9, 97.1, 96.5, 95.9},
      {98.4, 98.2, 98.1, 98.1, 97.8, 97.3, 97.2, 97.1, 97.3, 96.7, 100.0, 100.0},
      {98.4, 98.2, 95.1, 98.0, 98.0, 97.0, 96.9, 96.9, 96.9, 100.0, 100.0, 100.0},
      {98.4, 96.7, 94.0, 98.0, 97.9, 96.9, 96.8, 96.8, 96.8, 100.0, 100.0, 100.0},
      {98.4, 94.0, 95.9, 98.0, 97.8, 96.8, 96.8, 96.7, 100.0, 100.0, 100.0, 100.0},
      {97.6, 95.3, 95.7, 97.9, 97.7, 96.6, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {93.1, 94.3, 94.4, 97.9, 97.7, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {92.6, 92.7, 93.1, 98.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {90.7, 91.1, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {89.6, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
  },
  {
      // DERATE 01
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 94.9, 98.5, 98.3, 98.1},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 94.9, 98.5, 98.3, 98.1},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 94.9, 98.5, 98.3, 98.1},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 94.9, 98.5, 98.3, 98.1},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 94.9, 98.5, 98.3, 98.1},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 94.9, 98.4, 98.1, 97.6},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.6, 95.7, 97.9, 97.6, 97.2},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 90.5, 90.2, 97.5, 97.7, 97.3, 97.1},
      {91.2, 90.5, 90.5, 89.4, 89.0, 89.1, 89.6, 89.1, 98.0, 97.7, 97.3, 96.5},
      {91.2, 90.5, 90.5, 89.4, 89.0, 88.4, 87.8, 89.8, 97.9, 97.1, 96.5, 95.9},
      {91.2, 90.5, 90.5, 89.4, 88.6, 87.3, 86.3, 92.9, 97.3, 96.7, 100.0, 100.0},
      {91.2, 90.5, 87.5, 89.2, 88.5, 86.8, 85.9, 96.9, 96.9, 100.0, 100.0, 100.0},
      {91.2, 89.1, 86.2, 89.0, 88.0, 86.3, 87.9, 96.8, 96.8, 100.0, 100.0, 100.0},
      {91.2, 86.5, 87.9, 88.8, 87.7, 85.9, 96.8, 96.7, 100.0, 100.0, 100.0, 100.0},
      {90.5, 87.9, 87.8, 88.6, 87.6, 87.2, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {86.3, 87.1, 86.8, 88.9, 87.9, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {85.7, 85.4, 85.4, 89.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {83.7, 83.7, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {82.6, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
  },
  {
      // DERATE 02
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 94.9, 98.5, 98.3, 98.1},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 94.9, 98.5, 98.3, 98.1},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 94.9, 98.5, 98.3, 98.1},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 94.9, 98.5, 98.3, 98.1},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 94.9, 98.5, 98.3, 98.1},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 94.9, 98.4, 98.1, 97.6},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 86.3, 95.7, 97.9, 97.6, 97.2},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 81.0, 87.1, 97.5, 97.7, 97.3, 97.1},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.9, 79.7, 88.5, 98.0, 97.7, 97.3, 96.5},
      {84.5, 83.3, 82.9, 81.0, 80.2, 79.1, 77.8, 89.8, 97.9, 97.1, 96.5, 95.9},
      {84.5, 83.3, 82.9, 81.0, 79.7, 77.7, 79.3, 92.9, 97.3, 96.7, 100.0, 100.0},
      {84.5, 83.3, 80.2, 80.8, 79.3, 77.0, 82.7, 96.9, 96.9, 100.0, 100.0, 100.0},
      {84.5, 82.1, 79.1, 80.5, 78.9, 76.6, 87.9, 96.8, 96.8, 100.0, 100.0, 100.0},
      {84.5, 79.9, 80.7, 80.5, 78.8, 79.1, 96.8, 96.7, 100.0, 100.0, 100.0, 100.0},
      {83.8, 81.1, 80.6, 80.6, 78.9, 87.2, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {79.8, 80.2, 79.5, 80.7, 80.5, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {79.1, 78.5, 78.1, 80.5, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {77.2, 76.8, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {76.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
  },
  {
      // DERATE 03
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 94.9, 98.5, 98.3, 98.1},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 94.9, 98.5, 98.3, 98.1},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 94.9, 98.5, 98.3, 98.1},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 94.9, 98.5, 98.3, 98.1},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 94.9, 98.5, 98.3, 98.1},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 94.9, 98.4, 98.1, 97.6},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 86.3, 95.7, 97.9, 97.6, 97.2},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.2, 87.1, 97.5, 97.7, 97.3, 97.1},
      {78.0, 77.0, 76.1, 73.5, 71.9, 71.1, 76.8, 88.5, 98.0, 97.7, 97.3, 96.5},
      {78.0, 77.0, 76.1, 73.5, 71.9, 70.2, 77.6, 89.8, 97.9, 97.1, 96.5, 95.9},
      {78.0, 77.0, 76.1, 73.5, 71.4, 69.0, 79.3, 92.9, 97.3, 96.7, 100.0, 100.0},
      {78.0, 77.0, 73.6, 73.3, 71.3, 71.2, 82.7, 96.9, 96.9, 100.0, 100.0, 100.0},
      {78.0, 75.8, 72.7, 73.2, 71.1, 74.3, 87.9, 96.8, 96.8, 100.0, 100.0, 100.0},
      {78.0, 73.6, 74.0, 73.1, 70.9, 79.1, 96.8, 96.7, 100.0, 100.0, 100.0, 100.0},
      {77.4, 74.6, 73.8, 72.9, 73.1, 87.2, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {73.6, 73.7, 72.7, 72.9, 80.5, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {72.9, 72.0, 71.3, 73.6, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {71.0, 70.2, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
      {70.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0},
  }};

  static int dclFindIdx(const double* breakpoints, int count, double value) {
    if (value <= breakpoints[0]) return 0;
    if (value >= breakpoints[count - 1]) return count - 2;
    int idx = 1;
    while (idx < count - 1 && value >= breakpoints[idx]) idx++;
    return idx - 1;
  }

  static double bilinearLookup(const double* gridTat, const double* gridAlt, const double* table, int tatCount, int altCount, double tat, double alt) {
    int ti = dclFindIdx(gridTat, tatCount, tat);
    int ai = dclFindIdx(gridAlt, altCount, alt);
    double tFrac = (gridTat[ti + 1] - gridTat[ti]) == 0 ? 0 : (tat - gridTat[ti]) / (gridTat[ti + 1] - gridTat[ti]);
    double aFrac = (gridAlt[ai + 1] - gridAlt[ai]) == 0 ? 0 : (alt - gridAlt[ai]) / (gridAlt[ai + 1] - gridAlt[ai]);
    double v00 = table[ti * altCount + ai];
    double v10 = table[(ti + 1) * altCount + ai];
    double v01 = table[ti * altCount + (ai + 1)];
    double v11 = table[(ti + 1) * altCount + (ai + 1)];
    double v0 = v00 + (v10 - v00) * tFrac;
    double v1 = v01 + (v11 - v01) * tFrac;
    return v0 + (v1 - v0) * aFrac;
  }

  static double climbDerateFactor(int level, double tat, double altitude) {
    if (level < 0 || level > 3) return 100.0;
    return bilinearLookup(dclTatBreakpoints, dclAltBreakpoints, &dclThr[level][0][0], 19, 12, tat, altitude);
  }

  // Real per-TFLEX-temperature flex thrust tables, transcribed from the GP7270 FCOM (PER-THR-NRM-FTO,
  // TFLEX 10 through TFLEX 75 in 5C steps). Each chart covers a shrinking altitude range as TFLEX
  // rises (physically: high flex derates are only usable at low, cool airports). Columns match the
  // same 9 altitude breakpoints as the main `limits` table's TO section (rows 0-8). Missing
  // level/altitude combinations are marked with the NO_FLEX_DATA sentinel and fall back to that TO
  // row's own CP/LP/THR%@CP/THR%@LP (i.e. no extra flex benefit at that altitude for that TFLEX).
  static constexpr double NO_FLEX_DATA = -999;

  static constexpr double flexLevels[14] = {10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75};

  // [level][altCol] = {CP, LP, THR%@CP, THR%@LP}; altCol order matches limits[0..8]'s altitude.
  static constexpr double flexData[14][9][4] = {
      // TFLEX 10
      {{NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {5, 5, 94.8, 94.8},
       {0, 5, 92.0, 93.0}},
      // TFLEX 15
      {{NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {14, 14, 96.0, 96.0},
       {10, 14, 94.3, 95.9},
       {5, 14, 90.9, 95.2},
       {0, 14, 88.2, 94.6}},
      // TFLEX 20
      {{NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {18, 18, 95.5, 95.5},
       {14, 18, 93.1, 95.2},
       {10, 18, 91.3, 94.9},
       {5, 18, 87.2, 94.3},
       {0, 18, 85.1, 94.1}},
      // TFLEX 25
      {{NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {22, 24, 95.3, 96.8},
       {18, 24, 93.5, 96.1},
       {14, 24, 88.8, 95.3},
       {10, 24, 86.8, 95.2},
       {5, 24, 84.2, 95.2},
       {0, 24, 81.9, 94.5}},
      // TFLEX 30
      {{NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {28, 28, 97.6, 97.6},
       {26, 28, 94.3, 95.9},
       {22, 28, 92.1, 95.3},
       {18, 28, 88.4, 94.1},
       {14, 28, 84.3, 94.4},
       {10, 28, 83.2, 94.3},
       {5, 28, 80.9, 94.1},
       {5, 26, 77.4, 90.7}},
      // TFLEX 35
      {{34, 34, 97.0, 97.0},
       {30, 34, 94.0, 96.9},
       {26, 34, 90.8, 96.9},
       {22, 34, 87.3, 96.0},
       {18, 34, 84.0, 96.1},
       {14, 34, 80.9, 95.5},
       {10, 34, 79.8, 95.2},
       {5, 30, 76.1, 90.1},
       {0, 26, 72.0, 84.4}},
      // TFLEX 40
      {{32, 38, 93.7, 96.4},
       {30, 38, 90.7, 96.1},
       {26, 38, 86.8, 95.6},
       {22, 38, 84.0, 95.6},
       {18, 38, 80.8, 95.4},
       {14, 38, 77.3, 94.3},
       {10, 34, 74.8, 91.3},
       {5, 30, 71.1, 84.2},
       {0, 26, 67.1, 78.7}},
      // TFLEX 45
      {{32, 44, 90.2, 96.9},
       {30, 44, 86.8, 96.6},
       {26, 44, 83.7, 96.5},
       {22, 44, 80.9, 96.3},
       {18, 42, 77.2, 93.9},
       {14, 38, 72.9, 88.9},
       {10, 34, 70.1, 83.6},
       {5, 30, 66.4, 78.6},
       {0, 26, 62.4, 73.1}},
      // TFLEX 50
      {{34, 48, 86.3, 96.1},
       {30, 48, 83.6, 96.1},
       {26, 48, 80.7, 95.9},
       {22, 46, 77.0, 93.0},
       {18, 42, 72.8, 88.6},
       {14, 38, 68.7, 83.7},
       {10, 34, 65.5, 78.2},
       {5, 30, 61.9, 73.2},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA}},
      // TFLEX 55
      {{34, 54, 83.1, 97.0},
       {30, 54, 80.6, 96.8},
       {26, 50, 76.4, 92.2},
       {22, 46, 72.5, 87.6},
       {18, 42, 68.5, 83.4},
       {14, 38, 64.6, 78.8},
       {10, 34, 61.3, 73.1},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA}},
      // TFLEX 60
      {{34, 56, 79.8, 94.5},
       {30, 54, 76.0, 91.3},
       {26, 52, 72.0, 90.0},
       {22, 50, 68.2, 86.8},
       {18, 42, 64.5, 78.4},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA}},
      // TFLEX 65
      {{34, 56, 75.3, 89.2},
       {30, 54, 71.7, 86.1},
       {26, 52, 67.8, 84.8},
       {22, 50, 64.0, 81.7},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA}},
      // TFLEX 70
      {{34, 48, 71.0, 79.1},
       {30, 48, 67.5, 77.5},
       {26, 48, 63.7, 75.6},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA}},
      // TFLEX 75
      {{34, 48, 66.8, 74.4},
       {30, 48, 63.4, 72.9},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA},
       {NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA, NO_FLEX_DATA}},
  };

  static int altColIndex(double altitude) {
    static constexpr double altCols[9] = {-2000, 0, 2000, 4000, 6000, 8000, 10000, 12000, 14000};
    int idx = 0;
    for (int i = 1; i < 9; i++) {
      if (altitude >= altCols[i]) idx = i;
    }
    return idx;
  }

  // Real GP7270 flex thrust as a function of TFLEX (10-75C), altitude, and OAT - interpolates across
  // TFLEX level, altitude (nearest breakpoint), and the CP/LP ramp within each level.
  static double flexLookup(double flexTemp, double ambientTemp, double altitude) {
    const double clampedFlex = std::max(10.0, std::min(75.0, flexTemp));
    const int    altIdx      = altColIndex(altitude);

    int loLevel = 0;
    while (loLevel < 12 && flexLevels[loLevel + 1] <= clampedFlex) loLevel++;
    const int hiLevel = std::min(13, loLevel + 1);

    auto levelCn1 = [&](int level) -> double {
      double cp    = flexData[level][altIdx][0];
      double lp    = flexData[level][altIdx][1];
      double thrCp = flexData[level][altIdx][2];
      double thrLp = flexData[level][altIdx][3];
      if (cp == NO_FLEX_DATA) {
        // No published data for this TFLEX/altitude combo - fall back to the TO table's own
        // CP/LP/THR%@CP/THR%@LP for that altitude (no extra flex benefit).
        cp    = limits[altIdx][1];
        lp    = limits[altIdx][2];
        thrCp = limits[altIdx][3];
        thrLp = limits[altIdx][4];
      }
      if (ambientTemp <= cp) return thrCp;
      const double clampedOat = std::min(ambientTemp, lp);
      const double m          = (thrLp - thrCp) / (lp - cp);
      return thrCp + m * (clampedOat - cp);
    };

    const double loCn1 = levelCn1(loLevel);
    if (loLevel == hiLevel) return loCn1;
    const double hiCn1 = levelCn1(hiLevel);
    const double frac  = (clampedFlex - flexLevels[loLevel]) / (flexLevels[hiLevel] - flexLevels[loLevel]);
    return loCn1 + (hiCn1 - loCn1) * frac;
  }

  static double limitN1(int    type,             //
                        double altitude,         //
                        double ambientTemp,      //
                        double ambientPressure,  //
                        double flexTemp,         //
                        double packs,            //
                        double nacelle,          //
                        double wing              //
  ) {
    int    rowMin   = 0;
    int    rowMax   = 0;
    double mach     = 0;

    switch (type) {
      case 0:  // TO
        rowMin = 0;
        rowMax = 8;
        mach   = 0;
        break;
      case 1:  // GA
        rowMin = 9;
        rowMax = 18;
        mach   = 0.225;
        break;
      case 2:  // CLB
        rowMin = 19;
        rowMax = 30;
        if (altitude <= 10000) {
          mach = Fadec::cas2mach(250, ambientPressure);
        } else {
          mach = Fadec::cas2mach(300, ambientPressure);
          if (mach > 0.78)
            mach = 0.78;
        }
        break;
      case 3:  // MCT
        rowMin = 31;
        rowMax = 42;
        mach   = Fadec::cas2mach(230, ambientPressure);
        break;
    }

    int hiAltRow = 0;
    int loAltRow = 0;
    if (altitude <= limits[rowMin][0]) {
      hiAltRow = rowMin;
      loAltRow = rowMin;
    } else if (altitude >= limits[rowMax][0]) {
      hiAltRow = rowMax;
      loAltRow = rowMax;
    } else {
      hiAltRow = finder(altitude, rowMin);
      loAltRow = hiAltRow - 1;
    }

    const double cp      = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][1], limits[hiAltRow][1]);
    const double lp      = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][2], limits[hiAltRow][2]);
    const double cn1Flat = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][3], limits[hiAltRow][3]);
    const double cn1Last = Fadec::interpolate(altitude, limits[loAltRow][0], limits[hiAltRow][0], limits[loAltRow][4], limits[hiAltRow][4]);

    double cn1 = 0;
    if (flexTemp > 0 && type <= 1) {
      // Real GP7270 flex thrust: flexTemp selects which TFLEX chart, ambientTemp positions within it.
      cn1 = flexLookup(flexTemp, ambientTemp, altitude);
    } else {
      if (ambientTemp <= cp) {
        cn1 = cn1Flat;
      } else {
        const double m = (cn1Last - cn1Flat) / (lp - cp);
        const double b = cn1Last - m * lp;
        cn1            = (m * ambientTemp) + b;
      }
    }

    const double bleed = bleedTotal(type, altitude, ambientTemp, cp, lp, flexTemp, packs, nacelle, wing);

    return (cn1 * (std::sqrt)(EngineRatios::theta2(mach, ambientTemp))) + bleed;
  }
};

#endif  // FLYBYWIRE_AIRCRAFT_THRUSTLIMITS_GP7000_HPP
