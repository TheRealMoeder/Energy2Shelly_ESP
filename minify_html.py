import os
import re
Import("env")

# Pfad zu deiner exakten Header-Datei
TARGET_HEADER = os.path.join(env.get("PROJECT_DIR"), "src", "web", "html_home.h")

def minify_web_code(content, is_html=True):
    """Minimiert HTML, CSS und JS durch Entfernen von Kommentaren und Whitespace."""
    # 1. HTML-Kommentare entfernen
    content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)
    
    # 2. JS & CSS Multi-Line Kommentare entfernen
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    
    # 3. JS Single-Line Kommentare entfernen (nur wenn es kein HTTP-Link ist)
    # Verhindert das Zerstören von Links wie http:// oder ws://
    content = re.sub(r'(?<!:|https|http|ws)\/\/.*', '', content)
    
    # 4. Alle Newlines, Tabs und mehrfache Leerzeichen in ein einzelnes Leerzeichen kollabieren
    content = re.sub(r'\s+', ' ', content)
    
    # 5. Leerzeichen um HTML-Tags optimieren (> < statt >  <)
    content = re.sub(r'>\s+<', '><', content)
    
    return content.strip()

def process_inline_header(source, target, env):
    if not os.path.exists(TARGET_HEADER):
        print(f">>> [Minifier] Fehler: Datei '{TARGET_HEADER}' nicht gefunden.")
        return

    print(f">>> [Minifier] Optimiere Web-Assets direkt in {TARGET_HEADER}...")

    with open(TARGET_HEADER, 'r', encoding='utf-8') as f:
        file_content = f.read()

    # Regex zum Finden von C++ Raw-String-Literalen (z.B. R"=====(...)=====" oder R"rawliteral(...)rawliteral")
    # Gruppe 1: Der Delimiter (z.B. ===== oder rawliteral)
    # Gruppe 2: Der eigentliche HTML/JS/CSS Inhalt
    raw_string_pattern = r'R"([a-zA-Z0-9_=]*)\((.*?)\)\1"'
    
    original_size = len(file_content)

    def replace_literal(match):
        delimiter = match.group(1)
        inner_content = match.group(2)
        
        # Inhalt minifizieren
        minified_inner = minify_web_code(inner_content)
        
        # Wieder als gültiges C++ Raw-String-Literal zusammensetzen
        return f'R"{delimiter}({minified_inner}){delimiter}"'

    # Alle Vorkommen von Raw-Strings in der Datei suchen und ersetzen
    optimized_content = re.sub(raw_string_pattern, replace_literal, file_content, flags=re.DOTALL)

    # Datei nur überschreiben, wenn sich wirklich etwas verkleinert hat
    if len(optimized_content) < original_size:
        with open(TARGET_HEADER, 'w', encoding='utf-8') as f:
            f.write(optimized_content)
        print(f">>> [Minifier] Fertig! Datei optimiert (Vorher: {original_size}B | Nachher: {len(optimized_content)}B)")
    else:
        print(">>> [Minifier] Keine Änderungen nötig. Die Datei ist bereits minifiziert.")

# Hook registrieren, bevor das Programm gebaut wird
env.AddPreAction("buildprog", process_inline_header)
