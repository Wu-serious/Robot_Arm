set(srcs    I2C/I2C.c         PCA9685/PCA9685.c         MPU6050/MPU6050.c         Servo/Servo.c )

set(include_dirs        I2C         PCA9685         MPU6050         Servo )

set(requires
        driver)
 idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES "${requires}")

 component_compile_options(-O2 -Wno-error=format -Wno-format)
