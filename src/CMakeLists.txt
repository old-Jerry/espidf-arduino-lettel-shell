idf_component_register( SRCS 
                            "main.cpp"
                            "shell_port.cpp"  # 带相对路径
                        PRIV_REQUIRES 
                            letter-shell
                        INCLUDE_DIRS 
                            ""
)