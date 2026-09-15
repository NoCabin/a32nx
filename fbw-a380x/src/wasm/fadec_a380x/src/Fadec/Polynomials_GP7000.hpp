// Copyright (c) 2023-2024 FlyByWire Simulations
// SPDX-License-Identifier: GPL-3.0

#ifndef FLYBYWIRE_AIRCRAFT_POLYNOMIAL_GP7000_H
#define FLYBYWIRE_AIRCRAFT_POLYNOMIAL_GP7000_H

#include <cmath>

// GP7000 (2-spool: N1 fan/LP, N2 HP) transient/thermodynamic model. The start/shutdown/EGT/fuel-flow
// curve shapes are idle-normalized and reused from Polynomials_A380X since no public GP7000 engine
// deck exists; oilPressure is refit to the real N2-vs-oil-pressure points from EASA TCDS IM.E.026.
class Polynomial_GP7000 {
 public:
  static double startN2(double currentSimN2, double previousN2, double idleN2) {
    double normalizedN2 = currentSimN2 * 60.0 / idleN2;

    constexpr double coefficients[16] = {
        4.03649879e+00,   -9.41981960e-01,  1.98426614e-01,   -2.11907840e-02,
        1.00777507e-03,   -1.57319166e-06,  -2.15034888e-06,  1.08288379e-07,
        -2.48504632e-09,  2.52307089e-11,   -2.06869243e-14,  8.99045761e-16,
        -9.94853959e-17,  1.85366499e-18,   -1.44869928e-20,  4.31033031e-23,
    };

    double outN2 = 0;
    for (int i = 0; i < 16; i++) {
      outN2 += coefficients[i] * (std::pow)(normalizedN2, i);
    }
    outN2 *= currentSimN2;

    if (outN2 < previousN2) {
      outN2 = previousN2 + 0.002;
    }
    if (outN2 >= idleN2 + 0.1) {
      outN2 = idleN2 + 0.05;
    }

    return outN2;
  }

  static double startN1(double fbwN2, double idleN2, double idleN1) {
    double normalizedN2 = fbwN2 / idleN2;

    constexpr double coefficients[9] = {
        -2.2812156e-12, -5.9830374e+01, 7.0629094e+02,  -3.4580361e+03, 9.1428923e+03,
        -1.4097740e+04, 1.2704110e+04,  -6.2099935e+03, 1.2733071e+03,
    };

    double normalN1pre =
        (-2.4698087 * (std::pow)(normalizedN2, 3)) + (0.9662026 * (std::pow)(normalizedN2, 2)) + (0.0701367 * normalizedN2);
    double normalN1post = 0;
    for (int i = 0; i < 9; i++) {
      normalN1post += coefficients[i] * (std::pow)(normalizedN2, i);
    }

    if (normalN1post >= normalN1pre) {
      return normalN1post * idleN1;
    } else {
      return normalN1pre * idleN1;
    }
  }

  static double startFF(double fbwN2, double idleN2, double idleFF) {
    double normalizedN2 = fbwN2 / idleN2;
    double normalizedFF = 0;

    constexpr double coefficients[9] = {
        3.1110282e-12,  1.0804331e+02,  -1.3972629e+03, 7.4874131e+03, -2.1511983e+04,
        3.5957757e+04,  -3.5093994e+04, 1.8573033e+04,  -4.1220062e+03,
    };

    if (normalizedN2 > 0.37) {
      for (int i = 0; i < 9; i++) {
        normalizedFF += coefficients[i] * (std::pow)(normalizedN2, i);
      }
    }

    if (normalizedFF < 0) {
      normalizedFF = 0;
    }

    return normalizedFF * idleFF;
  }

  static double startEGT(double fbwN2, double idleN2, double ambientTemp, double idleEGT) {
    double normalizedN2 = fbwN2 / idleN2;
    double normalizedEGT = 0;

    if (normalizedN2 < 0.17) {
      normalizedEGT = 0;
    } else if (normalizedN2 <= 0.4) {
      normalizedEGT = (0.04783 * normalizedN2) - 0.00813;
    } else {
      double egtCoefficients[9] = {
          -6.8725167e+02, 7.7548864e+03,  -3.7507098e+04, 1.0147016e+05, -1.6779273e+05,
          1.7357157e+05,  -1.0960924e+05, 3.8591956e+04,  -5.7912600e+03,
      };
      for (int i = 0; i < 9; i++) {
        normalizedEGT += egtCoefficients[i] * (std::pow)(normalizedN2, i);
      }
    }

    return (normalizedEGT * (idleEGT - ambientTemp)) + ambientTemp;
  }

  static double startOilTemp(double fbwN2, double idleN2, double ambientTemperature) {
    if (fbwN2 < 0.79 * idleN2) {
      return ambientTemperature;
    }
    if (fbwN2 < 0.98 * idleN2) {
      return ambientTemperature + 5;
    }
    return ambientTemperature + 10;
  }

  static double shutdownN2(double previousN2, double deltaTime) {
    double decayRate = previousN2 < 30 ? -0.0515 : -0.08183;
    return previousN2 * (std::exp)(decayRate * deltaTime);
  }

  static double shutdownN1(double previousN1, double deltaTime) {
    double decayRate = previousN1 < 4 ? -0.08 : -0.164;
    return previousN1 * exp(decayRate * deltaTime);
  }

  static double shutdownEGT(double previousEGT, double ambientTemp, double deltaTime) {
    double threshold       = ambientTemp + 140;
    double decayRate       = previousEGT > threshold ? 0.0257743 : 0.00072756;
    double steadyStateTemp = previousEGT > threshold ? 135 + ambientTemp : 30 + ambientTemp;
    return steadyStateTemp + (previousEGT - steadyStateTemp) * exp(-decayRate * deltaTime);
  }

  static double correctedEGT(double cn1, double cff, double mach, double alt) {
    cff = cff / 3;

    double c_EGT[16] = {
        3.2636e+02,  0.0000e+00,  9.2893e-01,  3.9505e-02,  3.9070e+02,  -4.7911e-04, 7.7679e-03,  5.8361e-05,
        -2.5566e+00, 5.1227e-06,  1.0178e-07,  -7.4602e-03, 1.2106e-07,  -5.1639e+01, -2.7356e-03, 1.9312e-08,
    };

    return c_EGT[0]                             //
           + c_EGT[1]                           //
           + (c_EGT[2] * cn1)                   //
           + (c_EGT[3] * cff)                   //
           + (c_EGT[4] * mach)                  //
           + (c_EGT[5] * alt)                   //
           + (c_EGT[6] * (std::pow)(cn1, 2))    //
           + (c_EGT[7] * cn1 * cff)             //
           + (c_EGT[8] * cn1 * mach)            //
           + (c_EGT[9] * cn1 * alt)             //
           + (c_EGT[10] * (std::pow)(cff, 2))   //
           + (c_EGT[11] * mach * cff)           //
           + (c_EGT[12] * cff * alt)            //
           + (c_EGT[13] * (std::pow)(mach, 2))  //
           + (c_EGT[14] * mach * alt)           //
           + (c_EGT[15] * (std::pow)(alt, 2));  //
  }

  static double correctedFuelFlow(double cn1, double mach, double alt) {
    double c_Flow[21] = {
        -1.7630e+02, -2.1542e-01, 4.7119e+01,  6.1519e+02,  1.8047e-03,  -4.4554e-01, -4.3940e+01,
        4.0459e-05,  -3.2912e+01, -6.2894e-03, -1.2544e-07, 1.0938e-02,  4.0936e-01,  -5.5841e-06,
        -2.3829e+01, 9.3269e-04,  2.0273e-11,  -2.4100e+02, 1.4171e-02,  -9.5581e-07, 1.2728e-11,
    };

    double outCFF = c_Flow[0]                                   //
                    + c_Flow[1]                                 //
                    + (c_Flow[2] * cn1)                         //
                    + (c_Flow[3] * mach)                        //
                    + (c_Flow[4] * alt)                         //
                    + (c_Flow[5] * (std::pow)(cn1, 2))          //
                    + (c_Flow[6] * cn1 * mach)                  //
                    + (c_Flow[7] * cn1 * alt)                   //
                    + (c_Flow[8] * (std::pow)(mach, 2))         //
                    + (c_Flow[9] * mach * alt)                  //
                    + (c_Flow[10] * (std::pow)(alt, 2))         //
                    + (c_Flow[11] * (std::pow)(cn1, 3))         //
                    + (c_Flow[12] * (std::pow)(cn1, 2) * mach)  //
                    + (c_Flow[13] * (std::pow)(cn1, 2) * alt)   //
                    + (c_Flow[14] * cn1 * (std::pow)(mach, 2))  //
                    + (c_Flow[15] * cn1 * mach * alt)           //
                    + (c_Flow[16] * cn1 * (std::pow)(alt, 2))   //
                    + (c_Flow[17] * (std::pow)(mach, 3))        //
                    + (c_Flow[18] * (std::pow)(mach, 2) * alt)  //
                    + (c_Flow[19] * mach * (std::pow)(alt, 2))  //
                    + (c_Flow[20] * (std::pow)(alt, 3));        //

    return 2.8 * outCFF;
  }

  static double oilTemperature(double thermalEnergy, double previousOilTemp, double maxOilTemperature, double deltaTime) {
    double k  = 0.0001;
    double dt = thermalEnergy * deltaTime * 0.02;

    double t_steady = ((maxOilTemperature * k * deltaTime) + previousOilTemp) / (1 + (k * deltaTime));

    double oilTemp_out;
    if (t_steady + dt >= maxOilTemperature) {
      oilTemp_out = maxOilTemperature;
    } else if (t_steady + dt >= maxOilTemperature - 10) {
      oilTemp_out = (t_steady + dt) * 0.999997;
    } else {
      oilTemp_out = (t_steady + dt);
    }

    return oilTemp_out;
  }

  static double oilGulpPct(double thrust) {
    const double oilGulpCoefficients[3] = {20.1968848, -1.2270302e-6, 1.78442e-10};
    const double oilGulpPercentage =
        oilGulpCoefficients[0] + (oilGulpCoefficients[1] * thrust) + (oilGulpCoefficients[2] * (std::pow)(thrust, 2));
    return oilGulpPercentage / 100;
  }

  // Refit from EASA TCDS IM.E.026 N2(rpm)/oil-pressure(kPa) points (6620/172, 10500/428, 13060/702;
  // 100% N2 = 10998 rpm), converted to percent N2 and psi.
  static double oilPressure(double simN2) {
    const double oilPressureCoefficients[3] = {25.55, -0.6794, 0.01112};
    return oilPressureCoefficients[0] + (oilPressureCoefficients[1] * simN2) + (oilPressureCoefficients[2] * (std::pow)(simN2, 2));
  }
};

#endif  // FLYBYWIRE_AIRCRAFT_POLYNOMIAL_GP7000_H
