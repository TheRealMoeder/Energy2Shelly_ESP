import os
import re
Import("env")

TARGET_HEADER = os.path.join(env.get("PROJECT_DIR"), "src", "web", "html_home.h")

def minify_web_code(content):
    """Minimiert HTML, CSS und JS sicher ohne Look-behind-Fehler."""
    # 1. HTML-Kommentare entfernen (<!-- kommentar -->)
    content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)
    
    # 2. JS & CSS Multi-Line Kommentare entfernen (/* kommentar */)
    content = re.sub(r'/\*.*?\*/', '', content, flags=re.DOTALL)
    
    # 3. JS Single-Line Kommentare (//) sicher entfernen
    # Dieser Regex matcht Strings/URLs (wie "http://...") komplett, fängt sie ab 
    # und löscht nur die echten Kommentare am Zeilenende.
    def remove_js_comments(match):
        matched_str = match.group(0)
        if matched_str.startswith('//'):
            return '' # Es ist ein echter Kommentar -> löschen
        return matched_str # Es ist ein String oder eine URL -> behalten

    # Matcht entweder Strings in Anführungszeichen, Template-Literale ODER Single-Line Kommentare
    js_pattern = r'(?:"[^"\\]*(?:\\.[^"\\]*)*"|\'[^\'\\]*(?:\\.[^\'\\]*)*\'|`[^`\\]*(?:\\.[^`\\]*)*`|\/\/.*)'
    content = re.sub(js_pattern, remove_js_comments, content)
    
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

    # Findet alle C++ Raw-String-Literale wie R"=====(...)====="
    raw_string_pattern = r'R"([a-zA-Z0-9_=]*)\((.*?)\)\1"'
    
    original_size = len(file_content)

    def replace_literal(match):
        delimiter = match.group(1)
        inner_content = match.group(2)
        
        # Inhalt fehlerfrei minifizieren
        minified_inner = minify_web_code(inner_content)
        
        return f'R"{delimiter}({minified_inner}){delimiter}"'

    optimized_content = re.sub(raw_string_pattern, replace_literal, file_content, flags=re.DOTALL)

    if len(optimized_content) < original_size:
        with open(TARGET_HEADER, 'w', encoding='utf-8') as f:
            f.write(optimized_content)
        print(f">>> [Minifier] Fertig! Datei optimiert (Vorher: {original_size}B | Nachher: {len(optimized_content)}B)")
    else:
        print(">>> [Minifier] Keine Änderungen nötig. Die Datei ist bereits minifiziert.")

env.AddPreAction("buildprog", process_inline_header)
