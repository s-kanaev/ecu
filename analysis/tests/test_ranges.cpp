#include <types.hpp>
#include <undefined.hpp>
#include <pins.hpp>
#include <e591/impl.hpp>
#include <e591/memory-locations.hpp>

#include "adc.hpp"
#include "common.hpp"

#include <cstdlib>
#include <cassert>
#include <sstream>
#include <list>
#include <vector>
#include <map>
#include <string>
#include <sstream>

#define TEMPERATURE_CHARACTERISTIC(temp)  \
  ((double)((double)(10.0f) * ((temp) + 273) / (double)(1000.0f)))

#define REVERSE_TEMP_CHAR(voltage)  \
  ((((double)(voltage) * 1000.0f) / 10.0f) - 273)


namespace ADC_Mocks {
  static const double VoltageRail = 5.0f;

  void prepare() {
    {
      ADCMockT IgnitionSwitchADC;
      VectorWithIdx<word> Values{7};
      Values.V().push_back(makeADCValue(0.0, VoltageRail));
      Values.V().push_back(makeADCValue(1.0, VoltageRail));
      Values.V().push_back(makeADCValue(2.0, VoltageRail));
      Values.V().push_back(makeADCValue(2.57, VoltageRail));
      Values.V().push_back(makeADCValue(4.0, VoltageRail));
      Values.V().push_back(makeADCValue(3.0, VoltageRail));
      Values.V().push_back(makeADCValue(5.0, VoltageRail));
      auto It = IgnitionSwitchADC.emplace(
        std::make_pair(IGNITION_VOLTAGE_PIN, Values)
      );

      die(It.second, "Couldn't insert ADC Mock");

      ADCMock[TestName::IgnitionSwitchADC] = IgnitionSwitchADC;
    }

    {
      ADCMockT CoolantTemperatrueADC;
      VectorWithIdx<word> Values{130 - (-45) + 2};

//       for (int T = -70; T <= 200; ++T) {
      for (int T = -40; T <= 120; ++T) {
        Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(T), VoltageRail));
      }

//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(-45), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(-20), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(-10), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(0), VoltageRail));
//
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(8), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(10), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(20), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(40), VoltageRail));
//
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(60), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(71), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(80), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(85), VoltageRail));
//
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(90), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(100), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(102), VoltageRail));
//       Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(125), VoltageRail));

      auto It = CoolantTemperatrueADC.emplace(
        std::make_pair(COOLANT_TEMP_PIN, Values)
      );

      die(It.second, "Couldn't insert ADC Mock");

      ADCMock[TestName::CoolantTempADC2] = CoolantTemperatrueADC;
    }

    {
      ADCMockT CoolantTempADC;
      VectorWithIdx<word> Values{130 - (-45) + 2};

      for (int T = -40; T <= 120; ++T) {
        Values.V().push_back(makeADCValue(TEMPERATURE_CHARACTERISTIC(T), VoltageRail));
      }

      auto It = CoolantTempADC.emplace(
        std::make_pair(COOLANT_TEMP_PIN, Values)
      );

      die(It.second, "Couldn't insert ADC Mock");

      ADCMock[TestName::CoolantTempADC3] = CoolantTempADC;
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
             std::initializer_list<pin> Pins) {
  std::map<pin, VectorWithIdx<word> *> CurrentMocks;
  ssize_t Size = -1;
  for (pin P : Pins) {
    CurrentMocks[P] = &ADC_Mocks::getAllCurrentADCMocks(P);

    die(Size != static_cast<ssize_t>(CurrentMocks[P]->V().size()) || Size >= 0,
        "Sizes for mock ADC values should be the same");

    if (Size < 0)
      Size = CurrentMocks[P]->V().size();
  }

  while (Size) {
    TesterF();

    std::map<pin, word> ToTest;

    for (pin P : Pins)
      ToTest[P] = CurrentMocks[P]->get();

    CheckerF(ToTest);
    --Size;
  }
}

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
  static const double VoltageRail = ADC_Mocks::VoltageRail;

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
}

void test_60BA() {
  byte InputHigh, InputLow;
  byte OutputHigh, OutputLow;

  std::list<std::pair<word, word>> FuncTable;

  auto Test = [&] {
    word Input = COMPOSE_WORD(InputHigh, InputLow);
    OutputHigh = ((InputLow & 0xF0) >> 4) | (InputHigh << 4);
    OutputLow = (InputLow & 0x0F) << 4;
    word Output = COMPOSE_WORD(OutputHigh, OutputLow);

    FuncTable.emplace_back(Input, Output);
  };

  for (InputHigh = 0x00; InputHigh < 0x10; ++InputHigh) {
    for (InputLow = 0x00; InputLow < 0xFF; ++InputLow) {
      Test();
    }
    die(InputLow == 0xFF, "Fail!");
    Test(); // InputLow == 0xFF
  }

  std::stringstream InputS, OutputS;
  InputS << "Input = [";
  OutputS << "Output = [";

  auto It = FuncTable.begin();
  InputS << It->first;
  OutputS << It->second;

  for (; It != FuncTable.end(); ++It) {
    InputS << ", " << It->first;
    OutputS << ", " << It->second;
  }

  InputS << "];\n";
  OutputS << "];\n";

  fprintf(stderr, "%s", InputS.str().data());
  fprintf(stderr, "%s", OutputS.str().data());
}

template <typename _Container>
typename std::enable_if<
  std::is_same<typename _Container::value_type, double>::value ||
  std::is_same<typename _Container::value_type, float>::value
>::type matlabOutputArray(const _Container &V, const char *MVarName, FILE *OutF) {
  auto It = V.begin();

  die(It != V.end(), std::string("Empty vector for ") + MVarName);

  fprintf(OutF, "%s = [ %g", MVarName, static_cast<double>(*It));
  for (++It; It != V.end(); ++It)
    fprintf(OutF, ", %g", static_cast<double>(*It));
  fprintf(OutF, "];\n");
}

template <typename _Container>
typename std::enable_if<
  std::is_same<typename _Container::value_type, byte>::value ||
  std::is_same<typename _Container::value_type, word>::value ||
  std::is_same<typename _Container::value_type, quad>::value
>::type matlabOutputArray(const _Container &V, const char *MVarName, FILE *OutF) {
  auto It = V.begin();

  die(It != V.end(), std::string("Empty vector for ") + MVarName);

  fprintf(OutF, "%s = [ %u", MVarName, static_cast<unsigned int>(*It));
  for (++It; It != V.end(); ++It)
    fprintf(OutF, ", %u", static_cast<unsigned int>(*It));
  fprintf(OutF, "];\n");
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
              unADCValue(IgnPinADCV, ADC_Mocks::VoltageRail), (unsigned)(IgnPinADCV),
              (unsigned)(RAM[0x3F]), (unsigned)(CHECK_BIT_AT(RAM[0x22], 4)),
              (unsigned)(CHECK_BIT_AT(RAM[0x22], 5)), (unsigned)(RAM[0x3C]),
              (unsigned)GET_MEM_BYTE(FLASH, MINIMUM_IGNITION_VOLTAGE),
              (unsigned)GET_MEM_BYTE(FLASH, MAXIMUM_IGNITION_VOLTAGE)
      );
    }, { IGNITION_VOLTAGE_PIN });
  }

  switchToTest(TestName::CoolantTempADC2);

  {
    std::vector<double> Temperature;
    std::vector<byte> Ram3A;
    std::vector<byte> Ram3D;
    std::vector<word> ADCVal;

    FILE *OutF = stderr;
    testADC(&Inputs_Part1_CoolantTemp, [&, OutF](const std::map<pin, word> &Test) {
      word ADCV = Test.at(COOLANT_TEMP_PIN);

      ADCVal.push_back(ADCV);
      Temperature.push_back(REVERSE_TEMP_CHAR(unADCValue(ADCV, ADC_Mocks::VoltageRail)));
      Ram3A.push_back(RAM[0x3A]);
      Ram3D.push_back(RAM[0x3D]);

      fprintf(OutF, "Coolant temp = %g, Coolant temp V = %g, ADC = %04x, RAM[0x3A] = %02x = %u, "
              "RAM[0x3D] = %02x = %u, RAM[0x23].2 = %u, RAM[0x23].3 = %u\n",
              REVERSE_TEMP_CHAR(unADCValue(ADCV, ADC_Mocks::VoltageRail)),
              unADCValue(ADCV, ADC_Mocks::VoltageRail), (unsigned)(ADCV),
              (unsigned)(RAM[0x3A]), (unsigned)(RAM[0x3A]),
              (unsigned)(RAM[0x3D]), (unsigned)(RAM[0x3D]),
              (unsigned)(CHECK_BIT_AT(RAM[0x23], 2)),
              (unsigned)(CHECK_BIT_AT(RAM[0x23], 3))
      );
    }, { COOLANT_TEMP_PIN });

    matlabOutputArray(Temperature, "temp", OutF);
    matlabOutputArray(ADCVal, "temp_adc", OutF);
    matlabOutputArray(Ram3A, "ram_3a", OutF);
    matlabOutputArray(Ram3D, "ram_3d", OutF);
  }

  switchToTest(TestName::CoolantTempADC3);

  {
    std::vector<double> Temperature;
    std::vector<word> AdjustedCoolantTemps;
    std::vector<word> ADCVal;

    FILE *OutF = stderr;
    testADC(
      [&, OutF] {
        word CoolantTemp = ADC_10bit(COOLANT_TEMP_PIN);
        SET_MEM_BYTE(XRAM, ADC_COOLANT_TEMP, HIGH(CoolantTemp));
        bool CoolantTempNotInLimits = false;

        if (!FLASH[0x805D]) {
          if (HIGH(CoolantTemp) < FLASH[0x8057]) {
            // coolant_temp_less_than_low_limit
            SET_BIT_IN(RAM[0x23], 2);
            CLEAR_BIT_IN(RAM[0x23], 3);

            CoolantTempNotInLimits = true;
          } else
            if (HIGH(CoolantTemp) > FLASH[0x8058]) {
            // coolant_temp_larger_than_high_limit
            CLEAR_BIT_IN(RAM[0x23], 2);
            SET_BIT_IN(RAM[0x23], 3);

            CoolantTempNotInLimits = true;
          }
        }

        word AdjustedCoolantTemp;

        // CoolantTempNotInLimits = false in both examples
        if (!CoolantTempNotInLimits || FLASH[0x805D]) {
          CLEAR_BIT_IN(RAM[0x23], 2);
          CLEAR_BIT_IN(RAM[0x23], 3);

          // 1. AdjustedCoolantTemp: 0x2F84
          // 2. AdjustedCoolantTemp: 0x0000
          AdjustedCoolantTemp = GetAdcValueFromTableAndAdjustForCalculus(
              RNG_START_IDX(FLASH, COOLANT_TEMPERATURE_TABLE_1), CoolantTemp);
        } else if (CoolantTempNotInLimits) /* CoolantTempNotInLimits && !FLASH[0x805D] */ {
          AdjustedCoolantTemp = COMPOSE_WORD(FLASH[0x8A4B], 0);
        }
#if __E591_HOST_COMPILATION
        else {
          assert(false && "Shouldn't get here");
        }
#endif
        AdjustedCoolantTemps.push_back(AdjustedCoolantTemp);
      },
      [&, OutF](const std::map<pin, word> &Test) {
        word ADCV = Test.at(COOLANT_TEMP_PIN);

        ADCVal.push_back(ADCV);
        Temperature.push_back(REVERSE_TEMP_CHAR(unADCValue(ADCV, ADC_Mocks::VoltageRail)));
      },
      { COOLANT_TEMP_PIN });

    matlabOutputArray(Temperature, "temp", OutF);
    matlabOutputArray(ADCVal, "temp_adc", OutF);
    matlabOutputArray(AdjustedCoolantTemps, "adj_temp", OutF);
  }

  test_60BA();

  return 0;
}
