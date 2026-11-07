import gzip
import os

try:
    from SCons.Script import Import, File
    Import("env")
except Exception:
    env = None

# SCons-sicherer Pfad-Check
if env:
    # Holt das aktuelle Projektverzeichnis direkt aus der PlatformIO/SCons-Umgebung
    ROOT = env.subst("$PROJECT_DIR")
else:
    # Fallback für den Fall, dass das Skript manuell via 'python skript.py' gestartet wird
    ROOT = os.path.dirname(os.path.abspath(__file__))

WEB_DIR = os.path.join(ROOT, "src", "web")

ASSETS = {
    "webplotter_html": os.path.join(WEB_DIR, "webplotter.html"),
    "webconsole_html": os.path.join(WEB_DIR, "webconsole.html"),
    "update_error_html": os.path.join(WEB_DIR, "update_error.html"),
    "update_index_html": os.path.join(WEB_DIR, "update_index.html"),
    "update_success_html": os.path.join(WEB_DIR, "update_success.html"),
}


def generate_header(name, source, output):
    with open(source, "rb") as source_file:
        compressed = gzip.compress(source_file.read())

    guard = name.upper() + "_H"
    compressed_len = len(compressed)

    with open(output, "w", encoding="utf-8") as header:
        header.write("// Automatically generated; do not edit.\n")
        header.write(f"#ifndef {guard}\n")
        header.write(f"#define {guard}\n\n")
        # Generates needed length variable (e.g. webplotter_html_len)
        header.write(f"const uint32_t {name}_len = {compressed_len};\n")
        header.write(f"const char {name}[] PROGMEM = {{\n")

        for offset in range(0, compressed_len, 16):
            chunk = compressed[offset:offset + 16]
            hex_bytes = ", ".join(f"0x{byte:02x}" for byte in chunk)
            header.write(f"    {hex_bytes},\n")

        header.write("};\n\n")
        header.write(f"#endif // {guard}\n")


def main():
    outputs = {
        name: os.path.join(WEB_DIR, name + ".h")
        for name in ASSETS
    }

    if env:
        for output in outputs.values():
            env.Clean(output, output)

        if env.IsCleanTarget():
            print("[WEB-ASSETS] Clean target requested.")
            return

    for name, source in ASSETS.items():
        output = outputs[name]
        print(f"[WEB-ASSETS] Generating {output} from {source}")
        generate_header(name, source, output)

    print("[WEB-ASSETS] Done.")


main()
