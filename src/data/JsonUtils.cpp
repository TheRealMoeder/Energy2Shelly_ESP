#include <Arduino.h>
#include <ArduinoJson.h>

// JSON path navigation utility with array support
JsonVariant resolveJsonPath(JsonVariant variant, const char *path) {
  for (size_t n = 0; path[n]; n++) {
    // Not a full array support, but works for Shelly 3EM emeters array!
    if (path[n] == '[') {
      variant = variant[JsonString(path, n)][atoi(&path[n + 1])];
      path += n + 4;
      n = 0;
    }
    if (path[n] == '.') {
      variant = variant[JsonString(path, n)];
      path += n + 1;
      n = 0;
    }
  }
  return variant[path];
}
