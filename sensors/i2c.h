#ifndef AVC_I2C
#define AVC_I2C

#include <inttypes.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

int i2cSendByte(int fd, uint8_t devAddr, uint8_t dstReg, uint8_t byte);
int i2cReqBytes(int fd, uint8_t devAddr, uint8_t srcReg, void* dstBuf, size_t bytes);

#ifdef __cplusplus
}
#endif

#endif
