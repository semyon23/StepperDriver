import os
import shutil
import SCons.Script

env = SCons.Script.DefaultEnvironment()

def copy_binary(source, target, env):
    build_dir = env.subst("$BUILD_DIR")
    prog_name = env.subst("$PROGNAME")
    bin_path = os.path.join(build_dir, f"{prog_name}.bin")

    project_dir = env.get("PROJECT_DIR")
    target_path = os.path.join(project_dir, f"{prog_name}.bin")

    if os.path.exists(bin_path):
        shutil.copyfile(bin_path, target_path)
        print(f"\n[CUSTOM] SUCCESS: Binary copied to {target_path}\n")
    else:
        print(f"\n[CUSTOM] ERROR: Firmware file not found at {bin_path}\n")

env.AlwaysBuild(env.Alias("buildprog", "$BUILD_DIR/${PROGNAME}.bin", copy_binary))
