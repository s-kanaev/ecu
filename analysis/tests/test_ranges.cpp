#include <types.hpp>
#include <undefined.hpp>
#include <pins.hpp>
#include <e591/impl.hpp>
#include <e591/memory-locations.hpp>

#include "adc.hpp"
#include "common.hpp"

#include <cstdlib>
#include <cassert>
#include <iostream>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <sstream>

namespace ADC_Mocks {
  void prepare() {
    {
      ADCMockT IgnitionSwitchADC;
      VectorWithIdx<word> Values{6};
      Values.V().push_back(makeADCValue(0.0, 5.0));
      Values.V().push_back(makeADCValue(1.0, 5.0));
      Values.V().push_back(makeADCValue(2.0, 5.0));
      Values.V().push_back(makeADCValue(2.57, 5.0));
      Values.V().push_back(makeADCValue(3.0, 5.0));
      Values.V().push_back(makeADCValue(4.0, 5.0));
      Values.V().push_back(makeADCValue(5.0, 5.0));
      auto It = IgnitionSwitchADC.emplace(
        std::make_pair(IGNITION_VOLTAGE_PIN, Values)
      );

      die(It.second, "Couldn't insert ADC Mock");

      ADCMock[TestName::IgnitionSwitchADC] = IgnitionSwitchADC;
    }
  }

  word getCurrentADCMock(pin Pin) {
    auto It = ADCMock.find(::CurrentTestName);

    die(It != ADCMock.end(),
        std::string("Invalid current test name ") + getTestName());

    auto It2 = It->second.find(Pin);

    die(It2 != It->second.end(),
        std::string("Can't get pin ") + std::to_string(Pin) + " for test " + getTestName());

    return It2->second.get();
  }

  VectorWithIdx<word> &getAllCurrentADCMocks(pin Pin) {
    auto It = ADCMock.find(::CurrentTestName);

    die(It != ADCMock.end(),
        std::string("Invalid current test name ") + getTestName());

    auto It2 = It->second.find(Pin);

    die(It2 != It->second.end(),
        std::string("Can't get pin ") + std::to_string(Pin) + " for test " + getTestName());

    return It2->second;
  }
}

////////////////////////////////////////////////////////////////////////////////

void testMultiplySigned() {
  FILE *OutF = stdout;

  fprintf(OutF, "================= %s START\n", __func__);

  word P[0x100][0x100];

  for (unsigned int M1 = 0; M1 < 0x100; ++M1)
    for (unsigned int M2 = 0; M2 < 0x100; ++M2)
      P[M1][M2] = MultiplySigned(BYTE(M1), BYTE(M2));

  fprintf(OutF, "xx :");
  for (unsigned int X = 0; X <= 0xFF; ++X)
    fprintf(OutF, " %02x", X);
  fprintf(OutF, "\n");

  for (unsigned int M1 = 0; M1 <= 0xFF; ++M1) {
    fprintf(OutF, "%02x :", M1);

    for (unsigned int M2 = 0; M2 <= 0xFF; ++M2)
      fprintf(OutF, " %04x", (int)(P[M1][M2]));

    fprintf(OutF, "\n");
  }

  fprintf(OutF, "================= %s FINISH\n", __func__);
}

template <typename _TesterF, typename _CheckerF>
void testADC(_TesterF TesterF, _CheckerF CheckerF,
             size_t Count, std::initializer_list<pin> Pins) {
  std::map<pin, VectorWithIdx<word> *> CurrentMocks;
  for (pin P : Pins)
    CurrentMocks[P] = &ADC_Mocks::getAllCurrentADCMocks(P);

  while (Count) {
    TesterF();

    std::map<pin, word> ToTest;

    for (pin P : Pins)
      ToTest[P] = CurrentMocks[P]->get();

    CheckerF(ToTest);
    --Count;
  }
}

#define TEMPERATURE_CHARACTERISTIC(temp)  \
  ((double)((double)(10.0f) * ((temp) + 273) / (double)(1000.0f)))

#define REVERSE_TEMP_CHAR(voltage)  \
  ((((double)(voltage) * 1000.0f) / 10.0f) - 273)

template <typename _ResT>
struct ADCPreparedTemp {
  using ResT = _ResT;
  const double VoltageRail;
  const int TempOrig;
  const double Voltage;
  const word ADCVal;
  const double UnADCVoltage;
  const double TempADC;

  ResT Result;

  ADCPreparedTemp(int Temp, double VRail)
      : VoltageRail{VRail}, TempOrig{Temp},
        Voltage{TEMPERATURE_CHARACTERISTIC(Temp)},
        ADCVal{makeADCValue(Voltage, VoltageRail)},
        UnADCVoltage{unADCValue(ADCVal, VoltageRail)},
        TempADC{REVERSE_TEMP_CHAR(UnADCVoltage)}
  {}
};

struct Temp {
  word AdjustedForCalculus;
  byte Adjusted;
};

void testGetAdcValueFromTable(FILE *OutF) {
  static const double VoltageRail = 5.0f;

  fprintf(OutF, "%i = %g = %04x = %g\n%i = %g = %04x = %g\n%i = %g = %04x = %g\n",
          8, TEMPERATURE_CHARACTERISTIC(8),
          (unsigned)makeADCValue(TEMPERATURE_CHARACTERISTIC(8), VoltageRail),
          unADCValue(makeADCValue(TEMPERATURE_CHARACTERISTIC(8), VoltageRail), VoltageRail),
          71, TEMPERATURE_CHARACTERISTIC(71),
          (unsigned)makeADCValue(TEMPERATURE_CHARACTERISTIC(71), VoltageRail),
          unADCValue(makeADCValue(TEMPERATURE_CHARACTERISTIC(71), VoltageRail), VoltageRail),
          102, TEMPERATURE_CHARACTERISTIC(102),
          (unsigned)makeADCValue(TEMPERATURE_CHARACTERISTIC(102), VoltageRail),
          unADCValue(makeADCValue(TEMPERATURE_CHARACTERISTIC(102), VoltageRail), VoltageRail)
  );


  std::vector<ADCPreparedTemp<Temp>> ADCValues;
  ADCValues.emplace_back(-45, VoltageRail);
  ADCValues.emplace_back(-20, VoltageRail);
  ADCValues.emplace_back(-10, VoltageRail);
  ADCValues.emplace_back(0, VoltageRail);
  ADCValues.emplace_back(8, VoltageRail);
  ADCValues.emplace_back(10, VoltageRail);
  ADCValues.emplace_back(20, VoltageRail);
  ADCValues.emplace_back(40, VoltageRail);
  ADCValues.emplace_back(60, VoltageRail);
  ADCValues.emplace_back(71, VoltageRail);
  ADCValues.emplace_back(80, VoltageRail);
  ADCValues.emplace_back(85, VoltageRail);
  ADCValues.emplace_back(90, VoltageRail);
  ADCValues.emplace_back(100, VoltageRail);
  ADCValues.emplace_back(102, VoltageRail);
  ADCValues.emplace_back(125, VoltageRail);

  for (auto PrepADCVal : ADCValues) {
    PrepADCVal.Result.AdjustedForCalculus = GetAdcValueFromTableAndAdjustForCalculus(
        RNG_START_IDX(FLASH, COOLANT_TEMPERATURE_TABLE_1),
        PrepADCVal.ADCVal);
    PrepADCVal.Result.Adjusted = AdjustTemperature(PrepADCVal.Result.AdjustedForCalculus);

    fprintf(OutF, "Temp = %i, Voltage = %g, ADC = %04x => "
            "Adjusted for calculus %04x, Adjusted %02x\n",
            PrepADCVal.TempOrig, PrepADCVal.Voltage, (unsigned)PrepADCVal.ADCVal,
            (unsigned)PrepADCVal.Result.AdjustedForCalculus,
            (unsigned)PrepADCVal.Result.Adjusted);
  }

  // TODO
}

int main() {
  INIT_FLASH();
  INIT_RAM();
  INIT_XRAM();

  ADC_Mocks::prepare();

  std::stringstream SS;
  location::dumpRegisteredMemoryLocations(SS);
  fprintf(stderr, "%s\n", SS.str().data());

  testMultiplySigned();

  switchToTest(TestName::CoolantTempADC);

//   testADC(&Inputs_Part1_CoolantTemp, [](word MockValue) {
//     // TODO
//   }, { COOLANT_TEMP_PIN });

  testGetAdcValueFromTable(stdout);

  switchToTest(TestName::IgnitionSwitchADC);

  {
    FILE *OutF = stderr;
    testADC(&Inputs_Part1_IgnitionSwitchVoltage, [OutF](const std::map<pin, word> &Test) {
      word IgnPinADCV = Test.at(IGNITION_VOLTAGE_PIN);

      fprintf(OutF, "Ignition switch V = %g, ADC = %04x, RAM[0x3F] = %02x, "
              "RAM[0x22].4 = %u, RAM[0x22].5 = %u, RAM[0x3C] = %02x, "
              "thresholds: %02x .. %02x\n",
              unADCValue(IgnPinADCV, 5.0), (unsigned)(IgnPinADCV),
              (unsigned)(RAM[0x3F]), (unsigned)(CHECK_BIT_AT(RAM[0x22], 4)),
              (unsigned)(CHECK_BIT_AT(RAM[0x22], 5)), (unsigned)(RAM[0x3C]),
              (unsigned)GET_MEM_BYTE(FLASH, MINIMUM_IGNITION_VOLTAGE),
              (unsigned)GET_MEM_BYTE(FLASH, MAXIMUM_IGNITION_VOLTAGE)
      );
    }, 6, { IGNITION_VOLTAGE_PIN });
  }

  return 0;
}
