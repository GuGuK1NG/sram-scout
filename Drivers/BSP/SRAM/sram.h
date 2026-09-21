#ifndef __SRAM_H
#define __SRAM_H
#include "SYSTEM/sys/sys.h"


#define SRAM_FSMC_NEX           3         /* ʹ��FSMC_NE3��SRAM_CS,ȡֵ��Χֻ����: 1~4 */

#define SRAM_FSMC_BCRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2]       /* BCR�Ĵ���,����SRAM_FSMC_NEX�Զ����� */
#define SRAM_FSMC_BTRX          FSMC_Bank1->BTCR[(SRAM_FSMC_NEX - 1) * 2 + 1]   /* BTR�Ĵ���,����SRAM_FSMC_NEX�Զ����� */
#define SRAM_FSMC_BWTRX         FSMC_Bank1E->BWTR[(SRAM_FSMC_NEX - 1) * 2]      /* BWTR�Ĵ���,����SRAM_FSMC_NEX�Զ����� */

#define SRAM_SIZE_BYTES (1024U * 1024U)
#define SRAM_WINDOW_SIZE   ((uint32_t)0x04000000U)   
#define SRAM_NE4_BASE      ((uint32_t)0x6C000000U)   
#define SIG_GATE   ((uint16_t)0x1234U)   
#define SIG1       ((uint16_t)0xA5A5U)   
#define SIG2       ((uint16_t)0x5A5AU)   
#define BUS_ADDR_CNT 6



typedef enum {
    GATE_OK          =  0,
    GATE_ERR_ROUND1  = -1,   
    GATE_ERR_ROUND2  = -2,  
    GATE_ERR_ROUND3  = -3,   
} Gate_Status;					//门限返回值

typedef enum {
    ALIAS_YES   = 0,   
    ALIAS_NO    = 1,   
    ALIAS_GHOST = 2,   
} Alias_Result;	

typedef struct {
      uint32_t alias_cnt;
      uint32_t indep_cnt;
      uint32_t ghost_cnt;
      uint32_t pred_ok;     /* 算术预测(@1M)与实测(ALIAS)一致的探测点数 */
      uint32_t pred_bad;    /* 预测与实测不符的点数 —— 正常必须为 0 */
      uint32_t base;
      uint32_t step;
      uint32_t k_max;
} ScanStats;

typedef enum{
	BIT_OK     = 0,
    BIT_STUCK0 = 1,   /* 该位恒 0 */
    BIT_STUCK1 = 2,   /* 该位恒 1 */
    BIT_BRIDGE = 3,   /* 疑似与其它位短路 */
}__attribute__((mode(QI))) BitStatus;

typedef enum {
    SCAN_OK        = 0,
    SCAN_ERR_RANGE = -1,
} Scan_Status;

typedef struct {
	uint32_t addr; //地址
	uint32_t pass;	//通过个数
	BitStatus st[16];	//st[0]=D0
	uint16_t raw[16];	//原始数据，方便交叉对比
}BusAddrResult;

typedef struct{
	BusAddrResult a[BUS_ADDR_CNT];		//A0-A5
	uint32_t cnt;						//计数器，实际测量的个数
}BusReport;



#define SRAM_BASE_ADDR         (0X60000000 + (0X4000000 * (SRAM_FSMC_NEX - 1)))
extern SRAM_HandleTypeDef g_sram_handler;    /* SRAM��� */

void sram_init(void);
void sram_write(uint8_t *pbuf,uint32_t addr,uint32_t datalen);
void sram_read(uint8_t *pbuf,uint32_t addr,uint32_t datalen);
uint8_t sram_test_read(uint32_t addr);
void sram_test_write(uint32_t addr, uint8_t data);

//**阶段1测试代码**//
Gate_Status  SRAM_WriteChannel_SelfTest(void);
Scan_Status  SRAM_AliasScan_Param(uint32_t base, uint32_t step,
                                  uint32_t k_max, ScanStats *out);


//**阶段2测试代码**//
uint32_t sram_walk(uint32_t addr, BitStatus *out, uint16_t *raw,uint8_t mode);
Scan_Status sram_run_bus_matrix(void);   /* 阶段2.2: 六地址矩阵扫描 */
void sram_run_bus_matrix_print(const BusReport *rep, uint8_t mode); //结果打印
#endif