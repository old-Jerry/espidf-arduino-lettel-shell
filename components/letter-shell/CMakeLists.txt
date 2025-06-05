# 注册组件 并创建一个名为 letter_shell（默认用目录名）的静态库目标。接下来几行给它添属性
idf_component_register(
    #	源文件列表。这些 .c 会被一起编译进组件库。
    SRCS "src/shell.c"
         "src/shell_ext.c"
        #  "src/shell_port.cpp"
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
    #    给组件添加编译选项。这里是为了找到arduino.h 的头文件。因为 letter-shell 需要用到 Arduino 的一些功能（比如 Serial），所以需要包含 Arduino 的头文件路径。
    # PRIV_INCLUDE_DIRS "../arduino/cores/esp32"
    # REQUIRES 公共依赖-任何使用本组件的其他组件也可以继承这些依赖
    # REQUIRES arduino
    # PRIV_REQUIRE 私有依赖-只有当前组件自己可以使用这些依赖，其他组件无法使用这些依赖
    # PRIV_REQUIRES arduino  # 直接依赖 arduino 组件（PlatformIO/ESP-IDF 会自动处理路径）
)

#给刚才生成的组件库 (${COMPONENT_LIB}) 追加编译选项。这里等价于在所有源文件前面加宏定义：#define SHELL_CFG_USER "shell_cfg_user.h"。letter-shell 库内部会用到这个宏去 #include SHELL_CFG_USER，从而让你用自己的配置头覆盖默认配置。
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    -DSHELL_CFG_USER="shell_cfg_user.h"
    -DARDUINO=10819
)
