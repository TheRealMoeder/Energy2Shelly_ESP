import os
import re
Import("env")

def patch_wifimanager(source, target, env):
    # Determine path to WiFiManager.cpp
    wm_cpp_path = os.path.join(
        env.subst("$PROJECT_LIBDEPS_DIR"), 
        env.subst("$PIOENV"), 
        "WiFiManager", 
        "WiFiManager.cpp"
    )

    if not os.path.exists(wm_cpp_path):
        print(f"[PlatformIO-Script] Error: {wm_cpp_path} not found.")
        return

    with open(wm_cpp_path, "r", encoding="utf-8") as file:
        code = file.read()

    modified = False

   
    func_update_pattern = r"(void\s+WiFiManager::handleUpdateDone\s*\([^)]*\)\s*\{)"
    match_update = re.search(func_update_pattern, code)
    
    if match_update:
        start_idx = match_update.start()
        func_block = code[start_idx:start_idx+2000]
        target_line_update = 'page += FPSTR(HTTP_UPDATE_SUCCESS);'
        
        if 'id=\\"countdown-update\\"' not in func_block:
            javascript_redirect_update = (
                'page += FPSTR("<div class=\\"msg S\\"><strong>Update success!</strong><br/>'
                'The device is restarting. Redirecting in <span id=\\"countdown-update\\">20</span> seconds...</div>'
                '<script>'
                'var time = 20;'
                'var timer = setInterval(function() {'
                '  time--;'
                '  document.getElementById(\\"countdown-update\\").textContent = time;'
                '  if (time <= 0) {'
                '    clearInterval(timer);'
                '    window.location.href = \\"/\\";'
                '  }'
                '}, 1000);'
                '</script>");'
            )
            
            if target_line_update in func_block:
                func_block_modified = func_block.replace(target_line_update, javascript_redirect_update, 1)
                code = code[:start_idx] + func_block_modified + code[start_idx+2000:]
                modified = True
                print("[PlatformIO-Script] SUCCESS: WiFiManager.cpp updated with Update Redirect.")
            else:
                print("[PlatformIO-Script] Warning: Target line in handleUpdateDone not found.")
        else:
            print("[PlatformIO-Script] Info: Update Redirect already applied.")
    else:
        print("[PlatformIO-Script] Warning: handleUpdateDone() structure not found.")

  
    func_exit_pattern = r"(void\s+WiFiManager::handleExit\s*\(\s*\)\s*\{)"
    match_exit = re.search(func_exit_pattern, code)
    
    if match_exit:
        start_idx = match_exit.start()
        func_block = code[start_idx:start_idx+2000]
        
        
        target_line_exit = 'page += FPSTR(S_exiting); // @token exiting'
        
        if 'id=\\"countdown-exit\\"' not in func_block:
            javascript_redirect_exit = (
                'page += FPSTR(S_exiting); // @token exiting\n  ' # keep original text, then append redirect
                'page += FPSTR("<div class=\\"msg S\\"><strong>Closing Portal...</strong><br/>'
                'Returning to main application. Redirecting in <span id=\\"countdown-exit\\">20</span> seconds...</div>'
                '<script>'
                'var time = 20;'
                'var timer = setInterval(function() {'
                '  time--;'
                '  document.getElementById(\\"countdown-exit\\").textContent = time;'
                '  if (time <= 0) {'
                '    clearInterval(timer);'
                '    window.location.href = \\"/\\";'
                '  }'
                '}, 1000);'
                '</script>");'
            )
            
            if target_line_exit in func_block:
                func_block_modified = func_block.replace(target_line_exit, javascript_redirect_exit, 1)
                code = code[:start_idx] + func_block_modified + code[start_idx+2000:]
                modified = True
                print("[PlatformIO-Script] SUCCESS: WiFiManager.cpp updated with Exit Redirect.")
            else:
                print("[PlatformIO-Script] Warning: Target line in handleExit not found.")
        else:
            print("[PlatformIO-Script] Info: Exit Redirect already applied.")
    else:
        print("[PlatformIO-Script] Warning: handleExit() structure not found.")

    
    if modified:
        with open(wm_cpp_path, "w", encoding="utf-8") as file:
            file.write(code)

patch_wifimanager(None, None, env)


env.Append(CONTAINING_DIR=env.subst("$PROJECT_LIBDEPS_DIR"))
def compile_wrapper(target, source, env):
    patch_wifimanager(None, None, env)
    return None

env.AddPreAction("$BUILD_DIR/src/main.cpp.o", patch_wifimanager)