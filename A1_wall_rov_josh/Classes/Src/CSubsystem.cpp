#include "CSubSystem.h"

CSubsystem::CSubsystem(const std::string& label)
  : Name(label)
{
}

CSubsystem::~CSubsystem() {
}

const std::string& CSubsystem::GetName() const {
  return Name;
}
