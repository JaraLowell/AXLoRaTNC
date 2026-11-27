#include "native/Arduino.h"
#include "native/InternalFileSystem.h"
#include "util/nrf_preferences.h"
#include <cassert>

namespace axlora::util {
bool logEnabled() { return false; }
void setLogEnabled(bool) {}
}

int main() {
  Preferences settings;
  assert(settings.begin("axloratnc", false));
  assert(settings.getUShort("missing", 42) == 42);
  assert(settings.putUShort("value", 1234) == sizeof(uint16_t));
  assert(settings.getType("value") == PT_U16);
  assert(settings.getUShort("value", 0) == 1234);
  assert(settings.getUChar("value", 7) == 7);
  assert(settings.putString("manager", "N0CALL") == 6);
  char manager[16]{};
  assert(settings.getString("manager", manager, sizeof(manager)) == 7);
  assert(strcmp(manager, "N0CALL") == 0);
  settings.end();

  Preferences reloaded;
  assert(reloaded.begin("axloratnc", false));
  assert(reloaded.getUShort("value", 0) == 1234);
  assert(reloaded.putString("manager", "OTHER") == 5);
  InternalFS.failRename = true;
  assert(reloaded.putString("manager", "FAILED") == 0);
  assert(reloaded.getString("manager", manager, sizeof(manager)) == 6);
  assert(strcmp(manager, "OTHER") == 0);
  reloaded.end();

  Preferences readonly;
  assert(readonly.begin("axloratnc", true));
  assert(readonly.putUShort("value", 10) == 0);
  assert(readonly.getUShort("value", 0) == 1234);
}
