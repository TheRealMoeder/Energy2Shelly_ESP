import gzip
import os
import subprocess

from SCons.Script import Import

Import("env")

try:
    git_version = subprocess.check_output(
        ["git", "describe", "--tags", "--always"]
    ).strip().decode()
except Exception:
    git_version = "v0.0.0-unknown"

env.Append(CPPDEFINES=[("VERSION_BUILD", f'\\"{git_version}\\"')])

ROOT = env.subst("$PROJECT_DIR")
WEB_DIR = os.path.join(ROOT, "src", "web")

ASSETS = {
    "home_html": None,
    "webplotter_html": os.path.join(WEB_DIR, "webplotter.html"),
    "webconsole_html": os.path.join(WEB_DIR, "webconsole.html"),
    "update_error_html": os.path.join(WEB_DIR, "update_error.html"),
    "update_index_html": os.path.join(WEB_DIR, "update_index.html"),
    "update_success_html": os.path.join(WEB_DIR, "update_success.html"),
}


def get_build_version():
    for define in env.get("CPPDEFINES", []):
        if isinstance(define, (tuple, list)) and len(define) >= 2:
            if define[0] == "VERSION_BUILD":
                value = str(define[1])
                if value.startswith('\\"') and value.endswith('\\"'):
                    return value[2:-2]
                if value.startswith('"') and value.endswith('"'):
                    return value[1:-1]
                return value

        elif isinstance(define, str) and define.startswith("VERSION_BUILD="):
            value = define.split("=", 1)[1]
            if value.startswith('\\"') and value.endswith('\\"'):
                return value[2:-2]
            if value.startswith('"') and value.endswith('"'):
                return value[1:-1]
            return value

    raise RuntimeError(
        "VERSION_BUILD is missing from SCons CPPDEFINES. "
        "Check the PlatformIO build_flags configuration."
    )


def build_home_html():
    template_path = os.path.join(WEB_DIR, "home.html")

    with open(template_path, "r", encoding="utf-8") as template_file:
        html = template_file.read()

    if "{{VERSION_BUILD}}" not in html:
        raise RuntimeError(
            f"Missing {{VERSION_BUILD}} placeholder in {template_path}"
        )

    return html.replace("{{VERSION_BUILD}}", get_build_version()).encode("utf-8")


def generate_header(name, compressed, output_path):
    guard = f"{name.upper()}_H"

    with open(output_path, "w", encoding="utf-8") as header:
        header.write("// Automatically generated; do not edit.\n")
        header.write("#include <Arduino.h>\n\n")
        header.write(f"#ifndef {guard}\n")
        header.write(f"#define {guard}\n\n")
        header.write(f"const uint32_t {name}_len = {len(compressed)};\n")
        header.write(f"const char {name}[] PROGMEM = {{\n")

        for offset in range(0, len(compressed), 16):
            chunk = compressed[offset : offset + 16]
            bytes_as_hex = ", ".join(f"0x{byte:02x}" for byte in chunk)
            header.write(f"    {bytes_as_hex},\n")

        header.write("};\n\n")
        header.write(f"#endif // {guard}\n")


def main():
    outputs = {
        name: os.path.join(WEB_DIR, f"{name}.h")
        for name in ASSETS
    }

    for output_path in outputs.values():
        env.Clean(output_path, output_path)

    if env.IsCleanTarget():
        print("[WEB-ASSETS] Clean target requested.")
        return

    version = get_build_version()
    print(f"[WEB-ASSETS] Building assets for version {version}")

    for name, source_path in ASSETS.items():
        if name == "home_html":
            data = build_home_html()
        else:
            with open(source_path, "rb") as source_file:
                data = source_file.read()

        compressed = gzip.compress(data, mtime=0)
        output_path = outputs[name]
        generate_header(name, compressed, output_path)
        print(
            f"[WEB-ASSETS] Generated {output_path}: "
            f"{len(data)} bytes -> {len(compressed)} bytes"
        )

    print("[WEB-ASSETS] Done.")


main()
