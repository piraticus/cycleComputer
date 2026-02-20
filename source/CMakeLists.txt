add_executable(${Project}
    main.cpp
    btHelper.cpp
)

target_include_directories(${Project} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../headers
    ${CMAKE_CURRENT_LIST_DIR}/../include
)

target_link_libraries(${Project} 
    FreeRTOS-Kernel-Heap4
    pico_stdlib
    pico_btstack_cyw43
    pico_cyw43_arch_lwip_sys_freertos
    pico_btstack_ble
    pico_btstack_classic
    )

target_compile_definitions(${Project} INTERFACE
    CYW43_TASK_STACK_SIZE=1024
    NO_SYS=0
    CYW43_LWIP=1
    HAVE_LWIP=1 # stops btstack calling lwip_init
    )

pico_enable_stdio_usb(${Project} 0)
pico_enable_stdio_uart(${Project} 1)
pico_add_extra_outputs(${Project})
