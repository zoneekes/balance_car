#include "flash_storage.h"
#include <string.h>
#include "pid.h"

#define PID_FLASH_MAGIC 0x50494446UL // 'PIDF'

#pragma pack(push,1)
typedef struct {
    uint32_t magic;
    float vertical_kp;
    float vertical_kd;
    float speed_kp;
    float speed_ki;
    float steering_kp;
    float steering_kd;
    int32_t target_speed;
    int32_t target_angle;
    uint32_t checksum;
} pid_flash_t;
#pragma pack(pop)

// Simple checksum: sum of all 32-bit words except checksum
static uint32_t calc_checksum(const pid_flash_t *p)
{
    const uint32_t *w = (const uint32_t *)p;
    uint32_t sum = 0;
    // checksum is last field, so compute over (sizeof - 4) bytes
    size_t words = (sizeof(pid_flash_t) - sizeof(uint32_t)) / 4;
    for(size_t i=0;i<words;i++) sum += w[i];
    return sum;
}

int flash_save_pid_params(void)
{
    pid_flash_t buf;
    memset(&buf, 0xFF, sizeof(buf));
    buf.magic = PID_FLASH_MAGIC;
    // copy from current pid variables
    buf.vertical_kp = vertical_kp;
    buf.vertical_kd = vertical_kd;
    buf.speed_kp = speed_kp;
    buf.speed_ki = speed_ki;
    buf.steering_kp = steering_kp;
    buf.steering_kd = steering_kd;
    buf.target_speed = target_speed;
    buf.target_angle = target_angle;
    buf.checksum = calc_checksum(&buf);

    HAL_StatusTypeDef status;
    FLASH_EraseInitTypeDef EraseInit;
    uint32_t PageError = 0;

    // Unlock flash
    HAL_FLASH_Unlock();

    // Erase one page at FLASH_SAVE_ADDR
    EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInit.PageAddress = FLASH_SAVE_ADDR;
    EraseInit.NbPages = 1;

    status = HAL_FLASHEx_Erase(&EraseInit, &PageError);
    if(status != HAL_OK){
        HAL_FLASH_Lock();
        return -1;
    }

    // Program as words (32-bit)
    uint32_t addr = FLASH_SAVE_ADDR;
    const uint32_t *words = (const uint32_t *)&buf;
    size_t word_count = sizeof(buf)/4;
    for(size_t i=0;i<word_count;i++){
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[i]);
        if(status != HAL_OK){
            HAL_FLASH_Lock();
            return -2;
        }
        addr += 4;
    }

    HAL_FLASH_Lock();
    return 0;
}

int flash_load_pid_params(void)
{
    pid_flash_t *p = (pid_flash_t *)FLASH_SAVE_ADDR;
    if(p->magic != PID_FLASH_MAGIC) return -1;
    uint32_t cs = calc_checksum(p);
    if(cs != p->checksum) return -2;

    __disable_irq();
    vertical_kp = p->vertical_kp;
    vertical_kd = p->vertical_kd;
    speed_kp = p->speed_kp;
    speed_ki = p->speed_ki;
    steering_kp = p->steering_kp;
    steering_kd = p->steering_kd;
    target_speed = p->target_speed;
    target_angle = p->target_angle;
    __enable_irq();

    return 0;
}
