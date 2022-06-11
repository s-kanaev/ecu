#include <stdlib.h>

#include <mask_set.hpp>
#include "e591/memory-locations.hpp"
#include "e591/impl.hpp"

using namespace Reg;

int main(void) {
  S0CON::Inst = S0CON::Init;

  {
    Set<S0CON>{}.add<S0CON::REN0>();
    //M.add<>
  }

  if (Bit<S0CON, S0CON::REN0>{}.get()) {
    printf("Fail on bit\n");
    abort();
  }

  USE_MEM_BYTE(RAM, TOOTH_COUNTER) ToothCounter;

  byte X = ToothCounter;

  (void)COIL_PIN[0];
  (void)SET_CLEAR_MASKS_PER_COIL[0];

  return 0;
}
