#include "common.hpp"
#include "adc.hpp"

#include <include/types.hpp>
#include <include/undefined.hpp>

#include <string>

word ADC_10bit(pin Pin) {
  using namespace ADC_Mocks;
  auto It = ADCMock.find(CurrentTestName);

  die(It != ADCMock.end(),
      std::string("Invalid current test name ") + getTestName());

  auto It2 = It->second.find(Pin);

  die(It2 != It->second.end(),
      std::string("Can't get pin ") + std::to_string(Pin) + " for test " + getTestName());

  return It2->second.next();
}

byte ADC_8bit(pin Pin) {
  using namespace ADC_Mocks;
  auto It = ADCMock.find(CurrentTestName);

  die(It != ADCMock.end(),
      std::string("Invalid current test name ") + getTestName());

  auto It2 = It->second.find(Pin);

  die(It2 != It->second.end(),
      std::string("Can't get pin ") + std::to_string(Pin) + " for test " + getTestName());

  return HIGH(It2->second.next());
}


