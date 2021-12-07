#include "common.hpp"

TestName CurrentTestName;

std::string getTestName() {
#define CASE(x) case TestName::x : { return STRINGIFY(x); break; }
  switch (CurrentTestName) {
    CASE(CoolantTempADC);
    CASE(IntakeAirTempADC);
    CASE(IgnitionSwitchADC);
    CASE(CoolantTempADC2);
    default:
      die(false, "Invalid test name");
      return "N / A";
      break;
  }
#undef CASE
}

void switchToTest(const TestName &T) {
  CurrentTestName = T;
}


namespace ADC_Mocks {
  std::map<TestName, ADCMockT> ADCMock;
}
