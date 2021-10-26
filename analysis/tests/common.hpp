#pragma once

#include <include/types.hpp>
#include <include/binary_ops.hpp>

#include <vector>
#include <map>
#include <iostream>

template <typename _Type>
struct VectorWithIdx {
  using Type = _Type;
  using List = std::vector<Type>;

private:
  List MV;
  mutable int MIdx;

public:
  VectorWithIdx(size_t Size) : MV{}, MIdx{0} {
    MV.reserve(Size);
  }

  VectorWithIdx(VectorWithIdx &&Other)
    : MV{std::move(Other.MV)}, MIdx{Other.MIdx} {}

    VectorWithIdx(const VectorWithIdx &Other)
    : MV{Other.MV}, MIdx{Other.MIdx} {}

  const Type &next() const { return MV[MIdx++]; }

  Type next() { return MV[MIdx++]; }

  Type get() const { return MV[MIdx]; }

  List &V() { return MV; }
};

#define STRINGIFY(x) #x

enum class TestName : int {
  INIT = 0,
  CoolantTempADC = INIT,
  IntakeAirTempADC,
  IgnitionSwitchADC,
  TESTS_COUNT,
  NONE = -1
};

extern TestName CurrentTestName;

std::string getTestName();
void switchToTest(const TestName &T);

template <typename _MsgT>
void die(bool Cond, _MsgT Msg) {
  if (!Cond) {
    std::cerr << Msg << "\n";
    abort();
  }
}

namespace ADC_Mocks {
  using ValueListT = VectorWithIdx<word>; // std::vector<word>;
  using ADCMockT = std::map<pin, ValueListT>;
  extern std::map<TestName, ADCMockT> ADCMock;
}
