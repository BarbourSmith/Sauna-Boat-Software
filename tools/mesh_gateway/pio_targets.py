# PlatformIO targets for reaching a module through the handheld's USB port
# (see gateway.py). Add to a module's platformio.ini:
#
#   extra_scripts = post:<path to>/tools/mesh_gateway/pio_targets.py
#   custom_mesh_module = steering | navigation
#
# then:
#   pio run -t meshupload     build, then install over the mesh
#   pio run -t meshmonitor    show the modules' logs

import os

Import("env")


def find_gateway(start):
    path = start
    while True:
        candidate = os.path.join(path, "tools", "mesh_gateway", "gateway.py")
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(path)
        if parent == path:
            raise RuntimeError("tools/mesh_gateway/gateway.py not found above " + start)
        path = parent


gateway = find_gateway(env.subst("$PROJECT_DIR"))
module = env.GetProjectOption("custom_mesh_module")

env.AddCustomTarget(
    name="meshupload",
    dependencies="$BUILD_DIR/${PROGNAME}.bin",
    actions='"$PYTHONEXE" "%s" upload %s "$BUILD_DIR/${PROGNAME}.bin"' % (gateway, module),
    title="Upload over the mesh",
    description="Install this firmware through the handheld's USB port",
)

env.AddCustomTarget(
    name="meshmonitor",
    dependencies=None,
    actions='"$PYTHONEXE" "%s" monitor' % gateway,
    title="Monitor over the mesh",
    description="Show the modules' logs through the handheld's USB port",
)
