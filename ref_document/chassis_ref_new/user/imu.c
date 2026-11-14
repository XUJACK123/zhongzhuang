/* DRDY-gated BMI088 SPI2, PC0 accel CS, PC3 gyro CS, PB13 clock, PC1/PC2 data.
 * Based on HKU bsp_bmi088; no RTOS, DMA, heater or external AHRS dependency. */
#include "imu.h"
#include "main.h"
#include "robot_config.h"
#include "algorithm/rl.h"
#include <math.h>
#include <string.h>
Imu g_imu;
static SPI_HandleTypeDef spi;
static uint32_t consumed_edges;
static const float rotation[3][3]=ROBOT_IMU_ROTATION;
static bool transfer(bool accel, uint8_t reg, uint8_t *data, unsigned len, bool write) {
    uint8_t tx[8]={0},rx[8]={0}; unsigned skip=write ? 1 : (accel ? 2 : 1);
    if (len+skip>sizeof(tx)) return false;
    tx[0]=write ? reg : (uint8_t)(reg|0x80);
    if (write) memcpy(tx+1,data,len);
    HAL_GPIO_WritePin(GPIOC,accel ? GPIO_PIN_0 : GPIO_PIN_3,GPIO_PIN_RESET);
    bool ok=HAL_SPI_TransmitReceive(&spi,tx,rx,(uint16_t)(len+skip),1)==HAL_OK;
    HAL_GPIO_WritePin(GPIOC,accel ? GPIO_PIN_0 : GPIO_PIN_3,GPIO_PIN_SET);
    if (ok && !write) memcpy(data,rx+skip,len);
    return ok;
}
static bool read_reg(bool accel, uint8_t reg, uint8_t *d, unsigned n) { return transfer(accel,reg,d,n,false); }
static bool write_reg(bool accel, uint8_t reg, uint8_t v) {
    bool ok=transfer(accel,reg,&v,1,true); HAL_Delay(2); return ok;
}
bool imu_init(void) {
    __HAL_RCC_SPI2_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    /* PC2_C analog switch must be closed to connect the digital SPI2 MISO pad. */
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2,SYSCFG_SWITCH_PC2_CLOSE);
    HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3,SYSCFG_SWITCH_PC3_CLOSE);
    GPIO_InitTypeDef g={0}; g.Mode=GPIO_MODE_OUTPUT_PP; g.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    g.Pin=GPIO_PIN_0|GPIO_PIN_3; HAL_GPIO_WritePin(GPIOC,g.Pin,GPIO_PIN_SET); HAL_GPIO_Init(GPIOC,&g);
    g.Mode=GPIO_MODE_AF_PP; g.Alternate=GPIO_AF5_SPI2; g.Pin=GPIO_PIN_1|GPIO_PIN_2; HAL_GPIO_Init(GPIOC,&g);
    g.Pin=GPIO_PIN_13; HAL_GPIO_Init(GPIOB,&g);
    spi.Instance=SPI2; spi.Init.Mode=SPI_MODE_MASTER; spi.Init.Direction=SPI_DIRECTION_2LINES;
    spi.Init.DataSize=SPI_DATASIZE_8BIT; spi.Init.CLKPolarity=SPI_POLARITY_HIGH;
    spi.Init.CLKPhase=SPI_PHASE_2EDGE; spi.Init.NSS=SPI_NSS_SOFT;
    spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_32; /* PLL1Q=275MHz /32=8.59MHz <10MHz */
    spi.Init.FirstBit=SPI_FIRSTBIT_MSB; spi.Init.TIMode=SPI_TIMODE_DISABLE;
    spi.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE; spi.Init.NSSPMode=SPI_NSS_PULSE_DISABLE;
    spi.Init.NSSPolarity=SPI_NSS_POLARITY_LOW; spi.Init.FifoThreshold=SPI_FIFO_THRESHOLD_01DATA;
    spi.Init.MasterKeepIOState=SPI_MASTER_KEEP_IO_STATE_ENABLE;
    if (HAL_SPI_Init(&spi)!=HAL_OK) return false;
    uint8_t id=0; (void)read_reg(true,0,&id,1); HAL_Delay(5);
    if (!write_reg(true,0x7e,0xb6) || !write_reg(false,0x14,0xb6)) return false;
    HAL_Delay(50); (void)read_reg(true,0,&id,1); HAL_Delay(2);
    if (!read_reg(true,0,&id,1) || id!=0x1e) return false;
    if (!read_reg(false,0,&id,1) || id!=0x0f) return false;
    static const uint8_t acc[][2]={{0x7d,4},{0x7c,0},{0x40,0xac},{0x41,3}}; /* 24g */
    static const uint8_t gyr[][2]={{0x0f,0},{0x10,0x81},{0x11,0},{0x15,0x80}}; /* 2000dps */
    for (unsigned i=0;i<4;++i) if (!write_reg(true,acc[i][0],acc[i][1])) return false;
    for (unsigned i=0;i<4;++i) if (!write_reg(false,gyr[i][0],gyr[i][1])) return false;
    /* Range/ODR readback is mandatory: wrong range silently rescales policy input. */
    for (unsigned i=0;i<4;++i) if (!read_reg(true,acc[i][0],&id,1) || id!=acc[i][1]) return false;
    static const uint8_t gyro_mask[4]={7,15,0xa0,0x80}; /* ignore reserved readback bits */
    for (unsigned i=0;i<4;++i) if (!read_reg(false,gyr[i][0],&id,1) ||
        (id&gyro_mask[i])!=(gyr[i][1]&gyro_mask[i])) return false;
    /* Same INT3 mapping and rising PE12 input as HKU's DM-MC02 driver. */
    if (!write_reg(false,0x16,0x0c) || !write_reg(false,0x18,0x01)) return false;
    memset(&g_imu,0,sizeof(g_imu)); g_imu.quaternion[0]=1;
    g_imu.gravity[2]=-1; g_imu.initialized=true;
    consumed_edges=0;
    __HAL_RCC_GPIOE_CLK_ENABLE();
    g.Pin=GPIO_PIN_12; g.Mode=GPIO_MODE_IT_RISING; g.Pull=GPIO_NOPULL;
    g.Speed=GPIO_SPEED_FREQ_LOW; HAL_GPIO_Init(GPIOE,&g);
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_12);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn,5,0); HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    return true;
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
    if (pin==GPIO_PIN_12) ++g_imu.drdy_edges;
}
static void decode(const uint8_t raw[6], float scale, float body[3]) {
    float chip[3];
    for (unsigned i=0;i<3;++i) chip[i]=(float)(int16_t)((uint16_t)raw[2*i+1]<<8|raw[2*i])*scale;
    for (unsigned i=0;i<3;++i) {
        body[i]=0; for (unsigned j=0;j<3;++j) body[i]+=rotation[i][j]*chip[j];
    }
}
void imu_service(uint32_t now) {
    Imu *m=&g_imu;
    if (!m->initialized || (m->samples && now==m->tick)) return;
    uint8_t ar[6],gr[6];
    uint32_t edges=m->drdy_edges;
    if (edges==consumed_edges) return; /* freshness requires a real DRDY edge */
    consumed_edges=edges;
    if (!read_reg(true,0x12,ar,6) || !read_reg(false,0x02,gr,6)) { ++m->errors; return; }
    decode(ar,24*9.81f/32768,m->accel); decode(gr,2000*(RL_PI/180)/32768,m->omega);
    float norm=sqrtf(m->accel[0]*m->accel[0]+m->accel[1]*m->accel[1]+m->accel[2]*m->accel[2]);
    if (!isfinite(norm) || norm<0.1f || norm>250) { ++m->errors; return; }
    float dt=m->samples ? (float)(now-m->tick)*0.001f : 0.001f;
    uint32_t gap=m->samples ? now-m->tick : 1;
    if (gap>m->max_gap_ms) m->max_gap_ms=gap;
    m->tick=now; ++m->samples;
    if (!m->calibrated) {
        bool still=fabsf(norm-9.81f)<0.5f;
        for (unsigned i=0;i<3;++i) if (fabsf(m->omega[i])>0.08f) still=false;
        /* A stationary bias average does not require uniform 1ms spacing.
         * Use the same real gap bound as the live IMU freshness contract. */
        if (!still || gap>ROBOT_IMU_TIMEOUT_MS) {
            if (!still) ++m->motion_resets;
            if (gap>ROBOT_IMU_TIMEOUT_MS) ++m->cadence_resets;
            m->calibration_samples=0; memset(m->bias_sum,0,sizeof(m->bias_sum));
            memset(m->accel_sum,0,sizeof(m->accel_sum)); return;
        }
        for (unsigned i=0;i<3;++i) { m->bias_sum[i]+=m->omega[i]; m->accel_sum[i]+=m->accel[i]/norm; }
        if (++m->calibration_samples<2000) return;
        for (unsigned i=0;i<3;++i) m->bias[i]=m->bias_sum[i]/2000;
        float roll=atan2f(m->accel_sum[1],m->accel_sum[2]);
        float pitch=atan2f(-m->accel_sum[0],hypotf(m->accel_sum[1],m->accel_sum[2]));
        float cr=cosf(roll/2),sr=sinf(roll/2),cp=cosf(pitch/2),sp=sinf(pitch/2);
        m->quaternion[0]=cr*cp; m->quaternion[1]=sr*cp;
        m->quaternion[2]=cr*sp; m->quaternion[3]=-sr*sp; m->calibrated=true;
    }
    if (gap>ROBOT_IMU_TIMEOUT_MS) {
        m->calibrated=false; m->calibration_samples=0;
        memset(m->bias_sum,0,sizeof(m->bias_sum)); memset(m->accel_sum,0,sizeof(m->accel_sum));
        ++m->errors; return;
    }
    float *q=m->quaternion;
    float up[3]={2*(q[1]*q[3]-q[0]*q[2]),2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2])};
    float w[3];
    for (unsigned i=0;i<3;++i) { m->omega[i]-=m->bias[i]; w[i]=m->omega[i]; }
    if (fabsf(norm-9.81f)<1.0f) {
        float a[3]; for (unsigned i=0;i<3;++i) a[i]=m->accel[i]/norm;
        w[0]+=2*(a[1]*up[2]-a[2]*up[1]); w[1]+=2*(a[2]*up[0]-a[0]*up[2]);
        w[2]+=2*(a[0]*up[1]-a[1]*up[0]);
    }
    float d[4]={-q[1]*w[0]-q[2]*w[1]-q[3]*w[2],q[0]*w[0]+q[2]*w[2]-q[3]*w[1],
                q[0]*w[1]-q[1]*w[2]+q[3]*w[0],q[0]*w[2]+q[1]*w[1]-q[2]*w[0]};
    float n=0; for (unsigned i=0;i<4;++i) { q[i]+=d[i]*0.5f*dt; n+=q[i]*q[i]; }
    if (!isfinite(n) || n<0.1f) { m->initialized=false; ++m->errors; return; }
    n=1/sqrtf(n); for (unsigned i=0;i<4;++i) q[i]*=n;
    m->gravity[0]=-2*(q[1]*q[3]-q[0]*q[2]); m->gravity[1]=-2*(q[0]*q[1]+q[2]*q[3]);
    m->gravity[2]=-(1-2*(q[1]*q[1]+q[2]*q[2]));
    m->yaw=atan2f(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
}
