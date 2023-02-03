#ifndef __LOGGER__H__
#define __LOGGER__H__

#include <stdio.h>

#ifdef ENABLE_TRACES
    #define LOG(format, ...) printf("%s():%d: [ "format" ]\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
    #define LOGN(format, ...) printf("%s():%d: [ "format" ]", __FUNCTION__, __LINE__, ##__VA_ARGS__)
    #define LOGARR(format, data, data_len)  printf("%s():%d: [ %s ]", __FUNCTION__, __LINE__, format);              \
                                            for (int i = 0; i < data_len; i++) {                                    \
                                                printf("0x%02x ", data[i]);                                         \
                                                if ((i + 1) % 10 == 0) {                                            \
                                                    printf("\n");                                                   \
                                                }                                                                   \
                                            }

#else
    #define LOG(format, ...) do; while(0)
    #define LOGNL(format, ...) do; while(0)
    #define LOGARR(format, data, data_len) do; while(0)
#endif

static void dump_buffer(uint8_t* data, uint8_t data_len)
{
#ifdef ENABLE_TRACES
    uint8_t i = 0;
    uint8_t j = 0;
    for (i = 0; i < data_len; i++) {
        printf("0x%02x ", data[i]);
        if ((i + 1) % 10 == 0) {
            printf("\t|");
            for (j = i - 9; j < (i - 9) + 10; j++) {
                if (data[j] >= 32 && data[j] <= 126) {
                    printf("%c", data[j]);
                } else {
                    printf(".");
                }
            }
            printf("|\n");
        }
    }
    if ((i + 1) % 10 != 0) {
        printf("\n");
    }
#endif
}

#define CRITICAL(format, ...) printf("{ CRITICAL }%s():%d: [ "format" ]\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#define LOG_ERR() printf("%s():%d: ERROR\n", __FUNCTION__, __LINE__)
#define CHECK_RET_VALUE(foo, val) if ((foo) != (val)) { LOG_ERR(); }

#endif /* __LOGGER__H__ */
