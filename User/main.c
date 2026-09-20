/**
 ****************************************************************************************************
 * @file        main.c
 * @author      ����ԭ���Ŷ�(ALIENTEK)
 * @version     V1.0
 * @date        2021-11-04
 * @brief       �ⲿSRAM ʵ��
 * @license     Copyright (c) 2020-2032, �������������ӿƼ����޹�˾
 ****************************************************************************************************
 * @attention
 *
 * ʵ��ƽ̨:����ԭ�� ̽���� F407������
 * ������Ƶ:www.yuanzige.com
 * ������̳:www.openedv.com
 * ��˾��ַ:www.alientek.com
 * �����ַ:openedv.taobao.com
 *
 ****************************************************************************************************
 */

#include <stdio.h>                          /* sprintf */
#include "./SYSTEM/sys/sys.h"
#include "./SYSTEM/usart/usart.h"
#include "./SYSTEM/delay/delay.h"
#include "./BSP/LED/led.h"
#include "./BSP/LCD/lcd.h"
#include "./USMART/usmart.h"
#include "./BSP/KEY/key.h"
#include "./BSP/SRAM/sram.h"

#define LINE_H      16
#define COL_X       10
/* Text rows MUST advance by LINE_H. A separator, however, is now drawn as a
 * 2px horizontal rule (a SHAPE), so it may be followed by a smaller gap.
 * Previously the separators were '-' strings, which occupy a full text line,
 * and the +4 gaps made every neighbouring pair of rows overlap.            */
#define ROW_TITLE   (10)
#define ROW_SEP1    (ROW_TITLE   + LINE_H + 4)   /* rule */
#define ROW_WINDOW  (ROW_SEP1    + 4)
#define ROW_ALIAS   (ROW_WINDOW  + LINE_H)
#define ROW_CAP     (ROW_ALIAS   + LINE_H)
#define ROW_SEP2    (ROW_CAP     + LINE_H + 4)   /* rule */
#define ROW_SCAN1   (ROW_SEP2    + 4)
#define ROW_SCAN2   (ROW_SCAN1   + LINE_H)
#define ROW_VERDICT (ROW_SCAN2   + LINE_H)
#define ROW_SEP3    (ROW_VERDICT + LINE_H + 4)   /* rule */
#define ROW_HINT    (ROW_SEP3    + 4)

#define POS_X       10
#define POS_W       176
#define POS_THICK   2

/**
 * @brief       画一条 2px 水平分隔线
 * @note        用形状而不是字符串: 字符串要占满一整行文字高度,
 *              会把相邻两行挤在一起。
 */
static void draw_rule(uint16_t y)
{
    uint8_t i;

    for (i = 0; i < POS_THICK; i++)
    {
        lcd_draw_hline(POS_X, y + i, POS_W, BLACK);
    }
}


int main(void)
{
    uint8_t key;

    HAL_Init();                         /* ��ʼ��HAL�� */
    sys_stm32_clock_init(336, 8, 2, 7); /* ����ʱ��,168Mhz */
    delay_init(168);                    /* ��ʱ��ʼ�� */
    usart_init(115200);                 /* ���ڳ�ʼ��Ϊ115200 */
    usmart_dev.init(84);                /* ��ʼ��USMART */
    led_init();                         /* ��ʼ��LED */
    lcd_init();                         /* ��ʼ��LCD */
    key_init();                         /* ��ʼ������ */
    sram_init();                        /* SRAM��ʼ�� */

	
    lcd_show_string(COL_X, ROW_TITLE, 200, LINE_H, 16,
                    "SRAM SCOUT  v0.1", BLUE);
    draw_rule(ROW_SEP1);

    if (SRAM_WriteChannel_SelfTest() != GATE_OK) {
        printf("Write Pipe Error\r\n");
		lcd_show_string(COL_X, ROW_WINDOW, 200, LINE_H, 16, "GATE FAIL", RED);
        while (1) { 
			LED1_TOGGLE();
			delay_ms(100);
		}
    }

	lcd_show_string(COL_X, ROW_WINDOW, 200, LINE_H, 16,
                    "Window : 0x68000000", BLACK);
    /* Labelled as HYPOTHESIS: these two values are hard-coded expectations.
     * They are only promoted to a conclusion once the scan CONFIRMS them. */
    lcd_show_string(COL_X, ROW_ALIAS, 200, LINE_H, 16,
                    "Hyp.ALIAS : 1MB", BLACK);
    lcd_show_string(COL_X, ROW_CAP, 200, LINE_H, 16,
                    "Hyp.CAPAC.: 1MB", BLACK);
    draw_rule(ROW_SEP2);
    lcd_show_string(COL_X, ROW_HINT, 200, LINE_H, 16,
                    "KEY0:alias KEY1:bus", BLACK);

    while (1)
    {
        key = key_scan(0);
		if(key==KEY0_PRES){
			//KEY0对应阶段1测试
			ScanStats st1,st2;
			Scan_Status s1,s2;
			
			s1 = SRAM_AliasScan_Param(0x68000000, 0x80000, 63, &st1);
			s2 = SRAM_AliasScan_Param(0x68080000, 0x80000, 63, &st2);
			
			char buf[40];
            sprintf(buf, "Scan1: %luA %luI %luG p%lu/%lu",
                    (unsigned long)st1.alias_cnt,
                    (unsigned long)st1.indep_cnt,
                    (unsigned long)st1.ghost_cnt,
                    (unsigned long)st1.pred_ok,
                    (unsigned long)(st1.pred_ok + st1.pred_bad));
            lcd_show_string(COL_X, ROW_SCAN1, 200, LINE_H, 16, buf, BLACK);

            sprintf(buf, "Scan2: %luA %luI %luG p%lu/%lu",
                    (unsigned long)st2.alias_cnt,
                    (unsigned long)st2.indep_cnt,
                    (unsigned long)st2.ghost_cnt,
                    (unsigned long)st2.pred_ok,
                    (unsigned long)(st2.pred_ok + st2.pred_bad));
            lcd_show_string(COL_X, ROW_SCAN2, 200, LINE_H, 16, buf, BLACK);

            
            const char *verdict;
            uint16_t vcolor;
            if (s1 != SCAN_OK || s2 != SCAN_OK) {
                verdict = "REJECTED";
                vcolor = RED;
            } else if (st1.pred_bad == 0 && st2.pred_bad == 0 &&
                       st1.ghost_cnt == 0 && st2.ghost_cnt == 0 &&
                       st1.pred_ok > 0 && st1.alias_cnt == st2.alias_cnt) {
                verdict = "CONFIRMED";
                vcolor = GREEN;
            } else {
                verdict = "INCONSISTENT";
                vcolor = RED;
            }
            lcd_show_string(COL_X, ROW_VERDICT, POS_W, LINE_H, 16,
                            (char *)verdict, vcolor);
		}
		else if (key == KEY1_PRES) {
            //KEY1对应阶段1测试
            char buf[24];
            uint32_t pass = sram_run_walk1(0x68000000);

            sprintf(buf, "Bus   : %lu/16 OK", (unsigned long)pass);
            lcd_show_string(COL_X, ROW_SCAN1, 200, LINE_H, 16, buf,
                            (pass == 16u) ? GREEN : RED);
        }
		LED0_TOGGLE();
		delay_ms(200);
    }
}


