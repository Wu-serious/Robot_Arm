set(srcs
        Script/Script.c
        Kinematics/Kinematics.c)

set(include_dirs
        Script
        Kinematics)

set(requires
        driver
        Hardware)
idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "${include_dirs}"
                    REQUIRES "${requires}")
component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)
