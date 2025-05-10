cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espidf-arduino-wifiscan)
list(APPEND EXTRA_COMPONENT_DIRS components/letter-shell)#letter组件
# idf_build_set_property(COMPILE_OPTIONS "-DSHELL_CFG_USER=\"shell_cfg_user.h\"" APPEND)

# idf_component_register(
#     SRCS "lib/letter-shell/src/shell.c"
#          "lib/letter-shell/src/shell_ext.c"
#          "lib/letter-shell/src/shell_port.c"
#          "lib/letter-shell/src/shell_companion.c"
#          "lib/letter-shell/src/shell_cmd_list.c"
#     INCLUDE_DIRS "lib/letter-shell"
# )

# list(APPEND EXTRA_COMPONENT_DIRS lib/letter-shell)