
#ifndef __FS_TEST_H__
#define __FS_TEST_H__

//#define FS_MODULE_TEST
#ifdef FS_MODULE_TEST

#define FS_OFFSET_ADDRESS         0x11005000
#define FS_SECTOR_NUM							5

void ftcase_simple_write_test(void);
void ftcase_write_del_test(void);
void ftcase_write_del_and_ble_enable_test(void);
#endif

#define FS_EXAMPLE
#ifdef FS_EXAMPLE
void fs_example(void);
#endif

//#define FS_TIMING_TEST
#ifdef FS_TIMING_TEST
void fs_timing_test(void);
#endif

#endif
