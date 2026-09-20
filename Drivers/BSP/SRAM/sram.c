#include "./BSP/SRAM/sram.h"
#include "./SYSTEM/usart/usart.h"



SRAM_HandleTypeDef g_sram_handler;

void sram_init(void){
	GPIO_InitTypeDef gpio_init_struct;
	FSMC_NORSRAM_TimingTypeDef fsmc_readwritetim = {0};

    __HAL_RCC_FSMC_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();
	
	gpio_init_struct.Pin =GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
							GPIO_PIN_4|GPIO_PIN_5;
	gpio_init_struct.Mode = GPIO_MODE_AF_PP;
	gpio_init_struct.Pull = GPIO_NOPULL;
	gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio_init_struct.Alternate = GPIO_AF12_FSMC;
	
	HAL_GPIO_Init(GPIOF,&gpio_init_struct);
	
	gpio_init_struct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	HAL_GPIO_Init(GPIOF,&gpio_init_struct);
	
	gpio_init_struct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|
							GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10;
	HAL_GPIO_Init(GPIOG,&gpio_init_struct);	

	gpio_init_struct.Pin =GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
	HAL_GPIO_Init(GPIOD,&gpio_init_struct);
	
	g_sram_handler.Instance = FSMC_NORSRAM_DEVICE;
	g_sram_handler.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	g_sram_handler.Init.NSBank = FSMC_NORSRAM_BANK3;
	g_sram_handler.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	g_sram_handler.Init.WriteOperation     = FSMC_WRITE_OPERATION_ENABLE;
	g_sram_handler.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	g_sram_handler.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	
	fsmc_readwritetim.AddressSetupTime = 0x0F;
	fsmc_readwritetim.DataSetupTime = 60;
	fsmc_readwritetim.AccessMode = FSMC_ACCESS_MODE_A;
	HAL_SRAM_Init(&g_sram_handler, &fsmc_readwritetim, &fsmc_readwritetim);
	
}
void sram_write(uint8_t *pbuf,uint32_t addr,uint32_t datalen){
	for (; datalen != 0; datalen--)
    {
        *(volatile uint8_t *)(SRAM_BASE_ADDR + addr) = *pbuf;
        addr++;
        pbuf++;
    }
}

void sram_read(uint8_t *pbuf, uint32_t addr, uint32_t datalen)
{
    for (; datalen != 0; datalen--)
    {
        *pbuf++ = *(volatile uint8_t *)(SRAM_BASE_ADDR + addr);
        addr++;
    }
}

void sram_test_write(uint32_t addr, uint8_t data)
{
    sram_write(&data, addr, 1); /* д��1���ֽ� */
}

uint8_t sram_test_read(uint32_t addr)
{
    uint8_t data;
    sram_read(&data, addr, 1); /* ��ȡ1���ֽ� */
    return data;
}

Gate_Status SRAM_WriteChannel_SelfTest(void){
	volatile uint16_t *p = (volatile uint16_t *)SRAM_BASE_ADDR;
    uint16_t rd;
	*p= SIG_GATE;
	__DSB();
	rd=*p;
	if (rd != SIG_GATE) {
        printf("[GATE] FAIL round1: write=0x%04X, read=0x%04X @0x%08X\r\n",
               SIG_GATE, rd, (uint32_t)p);
        return GATE_ERR_ROUND1;
    }
	
	*p = SIG1;
    __DSB();
    rd = *p;

    if (rd != SIG1) {
        printf("[GATE] FAIL round2: write=0x%04X, read=0x%04X @0x%08X\r\n",
               SIG1, rd, (uint32_t)p);
        return GATE_ERR_ROUND2;
    }

    
    *p = SIG_GATE;
    __DSB();
    rd = *p;

    if (rd != SIG_GATE) {
        printf("[GATE] FAIL round3: write=0x%04X, read=0x%04X @0x%08X\r\n",
               SIG_GATE, rd, (uint32_t)p);
        return GATE_ERR_ROUND3;
    }

    printf("[GATE] PASS @0x%08X\r\n", SRAM_BASE_ADDR);
    return GATE_OK;
}

/* ---------------------------------------------------------------------------
 * Alias test for ONE probe point: bidirectional cross-write.
 *
 * Design rule: every verdict must be backed by POSITIVE EVIDENCE, never by
 * inference. "A still holds SIG1" alone is NOT evidence of independence --
 * it also happens when B's write never landed at all.
 *
 *   direction 1: write A, then write B, read A
 *     rd_a == SIG2                  -> B's value sits in A's cell
 *                                      => same cell              -> ALIAS
 *     rd_a == SIG1 && rd_b == SIG2  -> each kept its own value AND
 *                                      B's write did land        -> INDEP
 *     anything else                 -> B's write never landed    -> GHOST
 *
 *   direction 2: mirrored (write B, then write A, read B)
 *
 * Both directions must agree; a disagreement means the result is not
 * trustworthy, so it is reported as GHOST rather than silently accepted.
 * ------------------------------------------------------------------------- */
static Alias_Result alias_probe_word(volatile uint16_t *A, volatile uint16_t *B)
{
    uint16_t rd_a1, rd_b1, rd_a2, rd_b2;
    Alias_Result d1, d2;

    /* ---- direction 1: write A, then B, read A ---- */
    *A = SIG1;
    *B = SIG2;
    __DSB();
    rd_a1 = *A;
    rd_b1 = *B;

    if (rd_a1 == SIG2) {
        d1 = ALIAS_YES;                       /* same cell: B's write clobbered A */
    } else if (rd_a1 == SIG1 && rd_b1 == SIG2) {
        d1 = ALIAS_NO;                        /* independent, and B's write landed */
    } else {
        d1 = ALIAS_GHOST;                     /* B's write did not land */
    }

    /* ---- direction 2: write B, then A, read B ---- */
    *B = SIG2;
    *A = SIG1;
    __DSB();
    rd_b2 = *B;
    rd_a2 = *A;

    if (rd_b2 == SIG1) {
        d2 = ALIAS_YES;                       /* same cell: A's write clobbered B */
    } else if (rd_b2 == SIG2 && rd_a2 == SIG1) {
        d2 = ALIAS_NO;
    } else {
        d2 = ALIAS_GHOST;                     /* A's write did not land */
    }

    if (d1 == ALIAS_GHOST || d2 == ALIAS_GHOST) {
        return ALIAS_GHOST;                   /* a write did not land */
    }
    if (d1 != d2) {
        return ALIAS_GHOST;                   /* directions disagree -> untrustworthy */
    }
    return d1;
}

/**
 * @brief  alias / mirror scan (parameterised)
 * @param  base  : reference address A, e.g. 0x68000000
 * @param  step  : probe stride, e.g. 0x80000
 * @param  k_max : number of probe points, B = base + k*step, k = 1..k_max
 * @note   If the ALIAS set is unchanged after shifting the base address, the
 *         verdict depends only on (B - A) mod alias-period, i.e. "alias" is a
 *         property of the ADDRESS DIFFERENCE, not of any absolute address.
 */
Scan_Status SRAM_AliasScan_Param(uint32_t base, uint32_t step, uint32_t k_max,ScanStats *out)
{
    
	uint32_t max_offset = (base - SRAM_BASE_ADDR) + k_max * step;
    if (max_offset > SRAM_WINDOW_SIZE) {
        printf("[SCAN] REJECT: base=0x%08lX step=0x%lX k_max=%lu, "
               "max B would reach 0x%08lX (>= NE4 0x%08lX)\r\n",
               (unsigned long)base, (unsigned long)step,
               (unsigned long)k_max,
               (unsigned long)(base + k_max * step),
               (unsigned long)SRAM_NE4_BASE);
        if (out) {
            out->alias_cnt = 0;
            out->indep_cnt = 0;
            out->ghost_cnt = 0;
            out->pred_ok   = 0;
            out->pred_bad  = 0;
            out->base = base;
            out->step = step;
            out->k_max = k_max;
        }
        return SCAN_ERR_RANGE;
    }

	volatile uint16_t *A = (volatile uint16_t *)base;
    uint32_t alias = 0, indep = 0, ghost = 0;
    uint32_t pred_ok = 0, pred_bad = 0;
    uint32_t k;
	
    printf("[SCAN] base=0x%08lX step=0x%lX k=1..%lu\r\n",
           (unsigned long)base,
           (unsigned long)step,
           (unsigned long)k_max);

    for (k = 1; k <= k_max; k++) {
        volatile uint16_t *B = (volatile uint16_t *)(base + k * step);
        uint32_t off = k * step;
        Alias_Result res = alias_probe_word(A, B);
        const char *tag = "-";
        uint8_t pred_alias, real_alias;

        if (off % SRAM_SIZE_BYTES == 0) {
            tag = "@1M";                      /* exactly on a 1MB boundary */
        }

        
		
        pred_alias = (off % SRAM_SIZE_BYTES == 0) ? 1U : 0U;
        real_alias = (res == ALIAS_YES) ? 1U : 0U;
        if (pred_alias == real_alias) {
            pred_ok++;
        } else {
            pred_bad++;
            printf("  !! PREDICT MISMATCH @ k=%lu off=0x%07lX: "
                   "predicted %s, measured %s\r\n",
                   (unsigned long)k, (unsigned long)off,
                   pred_alias ? "ALIAS" : "INDEP",
                   real_alias ? "ALIAS" : "INDEP");
        }

        switch (res) {
        case ALIAS_YES:
            alias++;
            printf("  k=%2lu off=0x%07lX %-5s B=0x%08lX  ALIAS\r\n",
                   (unsigned long)k, (unsigned long)off, tag,
                   (unsigned long)(uint32_t)B);
            break;

        case ALIAS_NO:
            indep++;
            printf("  k=%2lu off=0x%07lX %-5s B=0x%08lX  INDEP\r\n",
                   (unsigned long)k, (unsigned long)off, tag,
                   (unsigned long)(uint32_t)B);
            break;

        default:
            ghost++;
            printf("  k=%2lu off=0x%07lX %-5s B=0x%08lX  GHOST\r\n",
                   (unsigned long)k, (unsigned long)off, tag,
                   (unsigned long)(uint32_t)B);
            break;
        }
    }
	if (out) {
        out->alias_cnt = alias;
        out->indep_cnt = indep;
        out->ghost_cnt = ghost;
        out->pred_ok   = pred_ok;
        out->pred_bad  = pred_bad;
        out->base      = base;
        out->step      = step;
        out->k_max     = k_max;
    }

    printf("[SCAN] done: alias=%lu indep=%lu ghost=%lu pred_ok=%lu pred_bad=%lu\r\n",
           (unsigned long)alias, (unsigned long)indep,
           (unsigned long)ghost, (unsigned long)pred_ok,
           (unsigned long)pred_bad);
	return SCAN_OK;
}



uint32_t sram_walk1(uint32_t addr, BitStatus *out, uint16_t *raw)
{
    volatile uint16_t *p = (volatile uint16_t *)addr;	//起始地址
    uint32_t pass = 0;
    uint32_t i;

    for (i = 0; i < 16; i++) {
        uint16_t pat = (uint16_t)(1u << i);				//位移，依次写入0x0001 0x0002...
		
        uint16_t rd;

        *p = pat;
        __DSB();
		
		
        rd = *p;
//		注入测试，测试工具的正确性
//		if(i==0){
//			rd=(uint16_t)0x0000; //将D0改为0
//		}
//		if (i == 5) {
//			rd |= (uint16_t)(1u << 2);  //将D5改为1
//		}
        if (raw != 0) {
            raw[i] = rd;
        }

        if (rd == pat) {
            out[i] = BIT_OK;
            pass++;
        } else if ((rd & pat) == 0u) {
            out[i] = BIT_STUCK0;    
        } else {
            out[i] = BIT_STUCK1;   
        }
    }

    return pass;									//返回成功的个数
}


//状态字节返回器，根据BitStatus枚举返回
static const char *bit_status_str(BitStatus s)
{
    switch (s) {
    case BIT_OK:     return "OK";
    case BIT_STUCK0: return "S0";
    case BIT_STUCK1: return "S1?";
    case BIT_BRIDGE: return "BR";
    default:         return "??";
    }
}


static void print_bit_list(uint16_t mask)
{
    uint32_t i;
    uint32_t shown = 0;

    for (i = 0; i < 16; i++) {
        if (mask & (uint16_t)(1u << i)) {
            printf("%sD%lu", (shown == 0u) ? "" : " ", (unsigned long)i);
            shown++;
        }
    }
    if (shown == 0u) {
        printf("(none)");
    }
}

//启动器，会运行sram_walk1()
uint32_t sram_run_walk1(uint32_t addr)
{
    BitStatus st[16];
    uint16_t  raw[16];
    uint32_t  pass;
    uint32_t  i;

   
    if (((addr & 1u) != 0u) ||
        (addr < SRAM_BASE_ADDR) ||
        ((addr + 2u) > (SRAM_BASE_ADDR + SRAM_WINDOW_SIZE))) {
        printf("[WALK1] REJECT: addr=0x%08lX is not a halfword-aligned address "
               "inside 0x%08lX~0x%08lX\r\n",
               (unsigned long)addr,
               (unsigned long)SRAM_BASE_ADDR,
               (unsigned long)(SRAM_BASE_ADDR + SRAM_WINDOW_SIZE - 1u));
        return 0;
    }

    pass = sram_walk1(addr, st, raw);

    printf("\r\n==== Stage 2.1: Walking-1 data bus check ====\r\n");
    printf("addr  : 0x%08lX\r\n\r\n", (unsigned long)addr);
    printf("bit  pattern   write   read    verdict\r\n");
    printf("---  --------  ------  ------  -------------------------\r\n");

    for (i = 0; i < 16; i++) {
        uint16_t pat   = (uint16_t)(1u << i);
        uint16_t extra = (uint16_t)(raw[i] & (uint16_t)(~pat));

        printf("D%-3lu 0x%04X    0x%04X  0x%04X  %-5s",
               (unsigned long)i, pat, pat, raw[i], bit_status_str(st[i]));

        if (extra != 0u) {
            printf("  unexpected 1s: ");
            print_bit_list(extra);
        }
        printf("\r\n");
    }

    printf("\r\nresult: %lu/16 passed", (unsigned long)pass);

    if (pass == 16u) {
        printf("  -> D0~D15 all normal\r\n");
    } else {
        printf("\r\nbad bits: ");
        for (i = 0; i < 16; i++) {
            if (st[i] != BIT_OK) {
                printf("D%lu(%s) ", (unsigned long)i, bit_status_str(st[i]));
            }
        }
        printf("\r\n");
    }

    return pass;
}