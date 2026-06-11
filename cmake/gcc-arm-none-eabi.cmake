set(CMAKE_SYSTEM_NAME               Generic)
set(CMAKE_SYSTEM_PROCESSOR          arm)

# Arm GNU 工具链查找。
# 优先使用 PATH 中的 arm-none-eabi-*，找不到时在配置阶段给出明确错误。
set(TOOLCHAIN_PREFIX arm-none-eabi-)

find_program(ARM_GCC     ${TOOLCHAIN_PREFIX}gcc)
find_program(ARM_GXX     ${TOOLCHAIN_PREFIX}g++)
find_program(ARM_OBJCOPY ${TOOLCHAIN_PREFIX}objcopy)
find_program(ARM_SIZE    ${TOOLCHAIN_PREFIX}size)

if(NOT ARM_GCC OR NOT ARM_GXX OR NOT ARM_OBJCOPY OR NOT ARM_SIZE)
    message(FATAL_ERROR
        "未找到 Arm GNU Toolchain，请将 arm-none-eabi-gcc/g++/objcopy/size 加入 PATH。")
endif()

set(CMAKE_C_COMPILER   ${ARM_GCC}     CACHE FILEPATH "Arm GNU C 编译器" FORCE)
set(CMAKE_ASM_COMPILER ${ARM_GCC}     CACHE FILEPATH "Arm GNU ASM 编译器" FORCE)
set(CMAKE_CXX_COMPILER ${ARM_GXX}     CACHE FILEPATH "Arm GNU CXX 编译器" FORCE)
set(CMAKE_LINKER       ${ARM_GXX}     CACHE FILEPATH "Arm GNU 链接器" FORCE)
set(CMAKE_OBJCOPY      ${ARM_OBJCOPY} CACHE FILEPATH "Arm GNU objcopy 工具" FORCE)
set(CMAKE_SIZE         ${ARM_SIZE}    CACHE FILEPATH "Arm GNU size 工具" FORCE)

set(CMAKE_EXECUTABLE_SUFFIX_ASM     ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_C       ".elf")
set(CMAKE_EXECUTABLE_SUFFIX_CXX     ".elf")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# MCU 编译参数
set(TARGET_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${TARGET_FLAGS}")
set(CMAKE_ASM_FLAGS "${CMAKE_C_FLAGS} -x assembler-with-cpp -MMD -MP")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fdata-sections -ffunction-sections")

set(CMAKE_C_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_C_FLAGS_RELEASE "-Os -g0")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set(CMAKE_CXX_FLAGS_RELEASE "-Os -g0")

set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -fno-rtti -fno-exceptions -fno-threadsafe-statics")

set(CMAKE_EXE_LINKER_FLAGS "${TARGET_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -T \"${CMAKE_SOURCE_DIR}/STM32G431XX_FLASH.ld\"")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --specs=nano.specs")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-Map=${CMAKE_PROJECT_NAME}.map -Wl,--gc-sections")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--print-memory-usage")
set(TOOLCHAIN_LINK_LIBRARIES "m")
