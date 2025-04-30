# 注册组件 并创建一个名为 letter_shell（默认用目录名）的静态库目标。接下来几行给它添属性
idf_component_register(
    #	源文件列表。这些 .c 会被一起编译进组件库。
    SRCS "src/shell.c"
         "src/shell_ext.c"
         "src/shell_port.c"
         "src/log.c"
         "src/shell_companion.c"
         "src/shell_cmd_list.c"
    #向 所有引用该组件的代码 暴露的头文件路径（-I）。放公开 API 的 .h 就应该位于这两个目录下。
    INCLUDE_DIRS "./include"
    #	声明 依赖：本组件要用到 ESP-IDF 自带的 driver 组件（比如 uart.h、gpio.h 等）。IDF 会自动帮你把 driver 的头文件、库一起链接进来
    REQUIRES
        driver
    #	把 linker fragment 文件交给 IDF 处理。shell.lf 里描述了自定义段（如 shellCommand → flash_rodata），编译系统会在生成最终链接脚本时把这段规则合并进去。
    LDFRAGMENTS "shell.lf"
)
#给刚才生成的组件库 (${COMPONENT_LIB}) 追加编译选项。这里等价于在所有源文件前面加宏定义：#define SHELL_CFG_USER "shell_cfg_user.h"。letter-shell 库内部会用到这个宏去 #include SHELL_CFG_USER，从而让你用自己的配置头覆盖默认配置。
target_compile_options(${COMPONENT_LIB} PRIVATE -DSHELL_CFG_USER="shell_cfg_user.h")
