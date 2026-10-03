import os
import requests
import gzip
from SCons.Script import Import

try:
    Import("env")
except Exception:
    pass
URL = "https://cdn.jsdelivr.net/npm/chart.js"
OUTPUT_FILE = "src/web/chart_js.h"
USE_GZIP = True  




def main():
    if env:
        env.Clean(OUTPUT_FILE, OUTPUT_FILE)
        
        if env.IsCleanTarget():
            print(f"[CHART.JS] Clean. remove '{OUTPUT_FILE}' if exists.")
            return

    if os.path.exists(OUTPUT_FILE):
        print(f"[CHART.JS] '{OUTPUT_FILE}' exists. Skip download.")
        return

    print(f"[CHART.JS] Start download: {URL}")
    try:
        response = requests.get(URL, timeout=10)
        if response.status_code != 200:
            print(f"[CHART.JS] Error during download: {response.status_code}")
            return
    except Exception as e:
        print(f"[CHART.JS] Network error during download: {e}")
        return

    js_content = response.text
    print(f"[CHART.JS] Successfully downloaded. Size: {len(js_content)} Bytes.")
    print(f"[CHART.JS] Generating C++ header file in '{OUTPUT_FILE}'...")

    os.makedirs(os.path.dirname(OUTPUT_FILE), exist_ok=True)

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f:
        f.write("// automatically generated file. do not edit manually!\n")
        f.write("#ifndef CHART_JS_H\n#define CHART_JS_H\n#include <Arduino.h>\n\n")

        if USE_GZIP:
            gzip_data = gzip.compress(js_content.encode("utf-8"))
            f.write(f"// Compressed UMD-Bundle (Gzip). Original: {len(js_content)} Bytes, Compressed: {len(gzip_data)} Bytes\n")
            f.write(f"const uint32_t chart_js_len = {len(gzip_data)};\n")
            f.write("const char chart_js[] PROGMEM = {\n    ")
            
            hex_bytes = [f"0x{b:02x}" for b in gzip_data]
            for i, hb in enumerate(hex_bytes):
                f.write(hb)
                if i < len(hex_bytes) - 1:
                    f.write(", " + ("\n    " if (i + 1) % 16 == 0 else ""))
            f.write("\n};\n")
        else:
            f.write("// Uncompressed UMD-Version\n")
            f.write("const char chart_js[] PROGMEM = R\"rawliteral(\n")
            f.write(js_content)
            f.write("\n)rawliteral\";\n")
        f.write("\n#endif // CHART_JS_H\n")

    print("[CHART.JS] Successfully completed!")


main()
