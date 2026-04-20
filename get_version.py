import subprocess

version = subprocess.check_output(
    ["git", "describe", "--tags", "--always"]
).strip().decode()

print(f'-D VERSION_BUILD=\\"{version}\\"')