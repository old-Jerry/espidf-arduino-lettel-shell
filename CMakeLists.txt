cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(espidf-arduino-letter-shell)
# idf_build_set_property(COMPILE_OPTIONS "-DSHELL_CFG_USER=\"shell_cfg_user.h\"" APPEND)

#使用相对路径指定组件目录-但是在根目录下的无需声明，编译器会自动查找
set(EXTRA_COMPONENT_DIRS
    components/letter-shell
)
# idf_component_register(
#     SRCS "lib/letter-shell/src/shell.c"
#          "lib/letter-shell/src/shell_ext.c"
#          "lib/letter-shell/src/shell_port.c"
#          "lib/letter-shell/src/shell_companion.c"
#          "lib/letter-shell/src/shell_cmd_list.c"
#     INCLUDE_DIRS "lib/letter-shell"
# )
list(APPEND EXTRA_COMPONENT_DIRS components/letter-shell)
# list(APPEND EXTRA_COMPONENT_DIRS lib/letter-shell)