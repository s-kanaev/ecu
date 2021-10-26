#pragma once

inline word makeADCValue(double Voltage, double VoltageRail) {
  static const int ADCLimit = 0x03FF;
  static const int Shift = 6;
  static const int ADCMask = ADCLimit - 1;

  double ADCValueF = (Voltage / VoltageRail) * ADCLimit;
  int ADCValue = ADCValueF;
  ADCValue &= ADCMask;
  ADCValue <<= Shift;

  return ADCValue;
}

inline double unADCValue(word ADCValue, double VoltageRail) {
  static const double ADCLimit = 0x03FF;
  static const int Shift = 6;

  return ((double)(ADCValue >> Shift) / ADCLimit) * VoltageRail;
}

