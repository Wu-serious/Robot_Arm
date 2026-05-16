set(srcs    
        LED/LED.c        
        I2C/I2C.c 
        PCA9685/PCA9685.c 
        MPU6050/MPU6050.c 
        Servo/Servo.c )

set(include_dirs
        LED 
        I2C 
        PCA9685     
        MPU6050
        Servo ) 

set(requires
        driver)
 idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES "${requires}")
                       
 component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)                      