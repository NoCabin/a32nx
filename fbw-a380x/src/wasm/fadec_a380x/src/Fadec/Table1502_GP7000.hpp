// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

#ifndef FLYBYWIRE_AIRCRAFT_TABLE1502_GP7000_HPP
#define FLYBYWIRE_AIRCRAFT_TABLE1502_GP7000_HPP

#include <cmath>

#include "Fadec.h"

// GP7000 idle/CN1-vs-CN2 lookup. Ground idle N2 is 63.4% per EASA TCDS IM.E.026 (GP7270); the
// CN1-vs-CN2 shape reuses Table1502_A380X's grid since no public GP7000 correlation chart exists.
class Table1502_GP7000 {
  static constexpr double table1502[13][4] = {
      {16.012,  0.000,   0.000,   17.000 },
      {19.355,  1.6253,  1.6253,  17.345 },
      {22.874,  2.1385,  2.1385,  18.127 },
      {50.147,  10.949,  10.949,  26.627 },
      {60.000,  16.299,  16.299,  33.728 },
      {67.742,  22.240,  22.240,  40.082 },
      {73.021,  26.877,  26.877,  43.854 },
      {78.299,  35.047,  35.047,  48.899 },
      {81.642,  43.625,  43.625,  53.557 },
      {85.337,  63.107,  63.107,  63.107 },
      {87.977,  74.757,  74.757,  74.757 },
      {97.800,  97.200,  97.200,  97.200 },
      {118.000, 115.347, 115.347, 115.347}
  };

 public:
  static double iCN2(double pressureAltitude, double mach) {
    return 63.4 / ((std::sqrt)((288.15 - (1.98 * pressureAltitude / 1000)) / 288.15) * (std::sqrt)(1 + (0.2 * (std::pow)(mach, 2))));
  }

  static double iCN1(double pressureAltitude, double mach, [[maybe_unused]] double ambientTemp) {
    const double cn2 = iCN2(pressureAltitude, mach);

    int i = 0;
    while (table1502[i][0] <= cn2 && i < 13) {
      i++;
    }

    const double cn2lo   = table1502[i - 1][0];
    const double cn2hi   = table1502[i][0];
    const double cn1lolo = table1502[i - 1][1];
    const double cn1hilo = table1502[i][1];
    const double cn1lohi = table1502[i - 1][3];
    const double cn1hihi = table1502[i][3];

    const double cn1_lo = Fadec::interpolate(cn2, cn2lo, cn2hi, cn1lolo, cn1hilo);
    const double cn1_hi = Fadec::interpolate(cn2, cn2lo, cn2hi, cn1lohi, cn1hihi);

    return Fadec::interpolate(mach, 0.2, 0.9, cn1_lo, cn1_hi);
  }
};

#endif  // FLYBYWIRE_AIRCRAFT_TABLE1502_GP7000_HPP
