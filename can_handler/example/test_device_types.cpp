#include <cstdio>
#include "../lib/pub_user.h"

int main()
{
    printf("=== Testing different device types ===\n\n");
    
    // 测试所有设备类型
    device_def_t types[] = {DEV_USB2CANFD, DEV_USB2CANFD_DUAL, DEV_ECAT2CANFD};
    const char* type_names[] = {"DEV_USB2CANFD", "DEV_USB2CANFD_DUAL", "DEV_ECAT2CANFD"};
    
    for (int i = 0; i < 3; i++) {
        printf("Testing %s...\n", type_names[i]);
        
        damiao_handle* handle = damiao_handle_create(types[i]);
        if (!handle) {
            printf("  Failed to create handle\n");
            continue;
        }
        
        damiao_print_version(handle);
        
        int device_cnt = damiao_handle_find_devices(handle);
        printf("  Found %d device(s)\n", device_cnt);
        
        if (device_cnt > 0) {
            device_handle* dev_list[16];
            int handle_cnt = 0;
            damiao_handle_get_devices(handle, dev_list, &handle_cnt);
            
            printf("  Got %d device handle(s)\n", handle_cnt);
            
            if (handle_cnt > 0) {
                printf("  *** SUCCESS with %s ***\n", type_names[i]);
                
                // 获取设备信息
                char strBuf[255] = {0};
                device_get_version(dev_list[0], strBuf, sizeof(strBuf));
                printf("  Device version: %s\n", strBuf);
                
                device_get_serial_number(dev_list[0], strBuf, sizeof(strBuf));
                printf("  Device SN: %s\n", strBuf);
            }
        }
        
        damiao_handle_destroy(handle);
        printf("\n");
    }
    
    return 0;
}
