#include "app_show.h"

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void APP_OLED_Show(void)
{
	/***** * * * * * * * * * * * * * * * * * 第一行 * * * * * * * * * * * * * * * * * *****/
	OLED_ShowString(0, 0, "X:");
	if (groundTruth[0].x < 0) {
		OLED_ShowChar(15, 0, '-', 12, 1), OLED_ShowNumber(25, 0, -groundTruth[0].x, 5, 12);
	} else {
		OLED_ShowChar(15, 0, '+', 12, 1), OLED_ShowNumber(25, 0, groundTruth[0].x, 5, 12);
	}
	OLED_ShowString(60, 0, "Y:");
	if (groundTruth[0].y < 0) {
		OLED_ShowChar(75, 0, '-', 12, 1), OLED_ShowNumber(85, 0, -groundTruth[0].y, 5, 12);
	} else {
		OLED_ShowChar(75, 0, '+', 12, 1), OLED_ShowNumber(85, 0, groundTruth[0].y, 5, 12);
	}

	/***** * * * * * * * * * * * * * * * * * 第二行 * * * * * * * * * * * * * * * * * *****/
	OLED_ShowString(0, 10, "X:");
	if (groundTruth[1].x < 0) {
		OLED_ShowChar(15, 10, '-', 12, 1), OLED_ShowNumber(25, 10, -groundTruth[1].x, 5, 12);
	} else {
		OLED_ShowChar(15, 10, '+', 12, 1), OLED_ShowNumber(25, 10, groundTruth[1].x, 5, 12);
	}
	OLED_ShowString(60, 10, "Y:");
	if (groundTruth[1].y < 0) {
		OLED_ShowChar(75, 10, '-', 12, 1), OLED_ShowNumber(85, 10, -groundTruth[1].y, 5, 12);
	} else {
		OLED_ShowChar(75, 10, '+', 12, 1), OLED_ShowNumber(85, 10, groundTruth[1].y, 5, 12);
	}

	/***** * * * * * * * * * * * * * * * * * 第三行 * * * * * * * * * * * * * * * * * *****/
	OLED_ShowString(0, 20, "X:");
	if (groundTruth[2].x < 0) {
		OLED_ShowChar(15, 20, '-', 12, 1), OLED_ShowNumber(25, 20, -groundTruth[2].x, 5, 12);
	} else {
		OLED_ShowChar(15, 20, '+', 12, 1), OLED_ShowNumber(25, 20, groundTruth[2].x, 5, 12);
	}
	OLED_ShowString(60, 20, "Y:");
	if (groundTruth[2].y < 0) {
		OLED_ShowChar(75, 20, '-', 12, 1), OLED_ShowNumber(85, 20, -groundTruth[2].y, 5, 12);
	} else {
		OLED_ShowChar(75, 20, '+', 12, 1), OLED_ShowNumber(85, 20, groundTruth[2].y, 5, 12);
	}
	/***** * * * * * * * * * * * * * * * * * 第四行 * * * * * * * * * * * * * * * * * *****/
	OLED_ShowString(0, 30, "m:");
	if (obs_x < 0) {
		OLED_ShowChar(15, 30, '-', 12, 1), OLED_ShowNumber(25, 30, -obs_x, 5, 12);
	} else {
		OLED_ShowChar(15, 30, '+', 12, 1), OLED_ShowNumber(25, 30, obs_x, 5, 12);
	}
	OLED_ShowString(60, 30, "n:");
	if (obs_y < 0) {
		OLED_ShowChar(75, 30, '-', 12, 1), OLED_ShowNumber(85, 30, -obs_y, 5, 12);
	} else {
		OLED_ShowChar(75, 30, '+', 12, 1), OLED_ShowNumber(85, 30, obs_y, 5, 12);
	}

	/***** * * * * * * * * * * * * * * * * * 第五行 * * * * * * * * * * * * * * * * * *****/
	OLED_ShowString(0, 40, "start:");
	OLED_ShowNumber(50, 40, startFlag, 1, 12);
	OLED_ShowString(65, 40, "mode:");
	OLED_ShowNumber(105, 40, advoidMode, 1, 12);

	/***** * * * * * * * * * * * * * * * * * 第六行 * * * * * * * * * * * * * * * * * *****/
	OLED_ShowString(0, 50, "Ftype:");
	OLED_ShowNumber(50, 50, formationType, 1, 12);
	if (CONTROL_MODE) {
		OLED_ShowString(65, 50, "Algo.");
	} else {
		OLED_ShowString(65, 50, "PS2");
	}

	OLED_Refresh_Gram();
}
