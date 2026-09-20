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

        /* Cross-check: the ARITHMETIC prediction (does this offset land on a
         * 1MB boundary?) against the HARDWARE verdict (is it really an alias?).
         * These two come from completely independent code paths, so agreement
         * on every point is strong evidence for the 1MB period.            */
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