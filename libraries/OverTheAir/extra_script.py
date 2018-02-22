Import("env")

#
# Dump build environment (for debug)
# print env.Dump()
#

#
# Upload actions
#

#def before_upload(source, target, env):
#    print "before_upload"
#    # do some actions


#def after_upload(source, target, env):
#    print "after_upload"
#    # do some actions

#print "Current build targets", map(str, BUILD_TARGETS)

#env.AddPreAction("upload", before_upload)
#env.AddPostAction("upload", after_upload)

#
# Custom actions when building program/firmware
#

#env.AddPreAction("buildprog", callback...)
#env.AddPostAction("buildprog", callback...)

#
# Custom actions for specific files/objects
#

#env.AddPreAction("$BUILD_DIR/firmware.elf", [callback1, callback2,...])
#env.AddPostAction("$BUILD_DIR/firmware.hex", callback...)

# custom action before building SPIFFS image. For example, compress HTML, etc.
#env.AddPreAction("$BUILD_DIR/spiffs.bin", callback...)

# custom action for project's main.cpp
#env.AddPostAction("$BUILD_DIR/src/main.cpp.o", callback...)

# Custom HEX from ELF
env.AddPostAction(
    "$BUILD_DIR/firmware.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex", "-R", ".eeprom",
        "$BUILD_DIR/firmware.elf", "$BUILD_DIR/firmware.hex"
    ]), "Building $BUILD_DIR/firmware.hex")
)
