#include "oled.h"
#include "oledfont.h"

extern I2C_HandleTypeDef hi2c1;     // 定义I2C通讯口
#define PAGE 4                      // 设置页数，128*64为8页，128*32为4页
uint8_t OLED_GRAM[PAGE][128] = {0}; // 图像缓存页数x128列

// 初始化命令
uint8_t CMD_Start[] = {
    // 0.91寸启动代码
    0xAE,       // 关闭显示
    0xA4,       // 显示模式(正常显示:0xA4 强制全亮:0xA5)
    0xA6,       // 设置显示方式(正常显示:0xA6 反相显示:0xA7)
    0x81, 0xC8, // 对比度设置(亮度调节) (1~255对应0x00~0xff)
    0x8D, 0x14, // 充电泵设置(启用:0x14 禁用:0x10)
    0xA1, 0xC8, // 左右反置关(段重映射),上下反置关(行重映射)
    0xA8, 0x1F, // 设置MUX数(显示行数) (64行:0x3F 32行:0x1F)
    0xD3, 0x00, // 设置垂直显示往上偏移，默认值00 没有偏移
    0xDA, 0x02, // 设置COM硬件引脚配置,适应分辨率(64行:0x12 32行:0x02)
};
// 寻址模式设置命令
uint8_t CMD_Frame[] = {

    // 水平/垂直专用
    0x20, 0x00,       // 设置内存地址模式：水平0x00，垂直0x01
    0x21, 0x00, 0x7f, // 设置列地址从0~127
    0x22, 0x00, 0x03, // 设置页地址从0到指定范围(64行:0x07 32行:0x03)

    // 页地址寻址专用
    // 0x20, 0x02,    // 设置内存地址模式：页寻址0x02
    // 0x00, 0x10,    // 设置列地址从0开始
    // 0xB0,          // 设置页地址从0开始(0-7对应0xB0-0xB7)

    0xAF // 开启显示
};

// 发送初始化命令
void OLED_Start(void)
{
    // 通过I2C发送，函数的0x00表示发送命令，0x40表示发送显示数据
    HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Start, sizeof(CMD_Start));
    HAL_Delay(10);
    HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Frame, sizeof(CMD_Frame));
}

// 发送整页数据
void OLED_Refreash(void)
{
    // 循环发送每行数据
    for (size_t i = 0; i < PAGE; i++) {
        // 等待i2c准备就绪
        for (size_t j = 0; j < 10000 && HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY; j++);
        // 发送一整行数据
        HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, OLED_GRAM[i], 128);
    }
}

// 初始化oled屏幕函数
void OLED_Init(void)
{
    HAL_Delay(300); // 这里的延时很重要
    OLED_Start();
    OLED_Clear();
    OLED_Refreash();
}

// 清屏
void OLED_Clear(void)
{
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 128; j++) {
            OLED_GRAM[i][j] = 0x00;
        }
    }
}

// 画点
//  x:0~127
//  y:0~31
//  t:1 填充 0,清空
void OLED_DrawPoint(uint8_t x, uint8_t y, uint8_t t)
{
    uint8_t i, m, n;
    i = y / 8;
    m = y % 8;
    n = 1 << m;
    if (t) {
        OLED_GRAM[i][x] |= n;
    } else {
        OLED_GRAM[i][x] = ~OLED_GRAM[i][x];
        OLED_GRAM[i][x] |= n;
        OLED_GRAM[i][x] = ~OLED_GRAM[i][x];
    }
}

// 画线
//  x1,y1:起点坐标
//  x2,y2:结束坐标
//  mode:1 填充 0,清空
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; // 计算坐标增量
    delta_y = y2 - y1;
    uRow    = x1; // 画线起点坐标
    uCol    = y1;
    if (delta_x > 0)
        incx = 1; // 设置单步方向
    else if (delta_x == 0)
        incx = 0; // 垂直线
    else {
        incx    = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // 水平线
    else {
        incy    = -1;
        delta_y = -delta_x;
    }
    if (delta_x > delta_y)
        distance = delta_x; // 选取基本增量坐标轴
    else
        distance = delta_y;
    for (t = 0; t < distance + 1; t++) {
        OLED_DrawPoint(uRow, uCol, mode); // 画点
        xerr += delta_x;
        yerr += delta_y;
        if (xerr > distance) {
            xerr -= distance;
            uRow += incx;
        }
        if (yerr > distance) {
            yerr -= distance;
            uCol += incy;
        }
    }
}

// 画水平虚线(像素坐标，左上为基点，右下增)
//  XS      行起始坐标(0~127)
//  XE      行终止坐标(0~127)
//  Y       列坐标(0~31)
//  Len     虚线单个线宽度
void OLED_DrawDashline(uint8_t XS, uint8_t XE, uint8_t Y, uint8_t Len)
{
    for (size_t i = XS; i <= XE; i += 8)
        OLED_DrawLine(i, Y, i + Len, Y, 1);
}

// x：圆心x坐标（0~127）
// y：圆心y坐标（0~31）
// r:圆的半径
void OLED_DrawCircle(uint8_t x, uint8_t y, uint8_t r)
{
    int a, b, num;
    a = 0;
    b = r;
    while (2 * b * b >= r * r) {
        OLED_DrawPoint(x + a, y - b, 1);
        OLED_DrawPoint(x - a, y - b, 1);
        OLED_DrawPoint(x - a, y + b, 1);
        OLED_DrawPoint(x + a, y + b, 1);

        OLED_DrawPoint(x + b, y + a, 1);
        OLED_DrawPoint(x + b, y - a, 1);
        OLED_DrawPoint(x - b, y - a, 1);
        OLED_DrawPoint(x - b, y + a, 1);

        a++;
        num = (a * a + b * b) - r * r; // 计算画的点离圆心的距离
        if (num > 0) {
            b--;
            a--;
        }
    }
}

// 在指定位置显示一个字符,包括部分字符
//  x：起点x坐标（0~127）
//  y：起点y坐标（0~3）
//  chr:字符
//  Char_Size:选择字体 16/12
//  Is_Reverse:0正常显示,1反白显示
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size, uint8_t Is_Reverse)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; // 得到偏移后的值
    if (x > 128 - 1) {
        x = 0;
        y = y + 2;
    }
    if (Char_Size == 16) {
        if (Is_Reverse == 0) {
            for (i = 0; i < 8; i++) {
                OLED_GRAM[y][x + i]     = F8X16[c * 16 + i];
                OLED_GRAM[y + 1][x + i] = F8X16[c * 16 + i + 8];
            }
        } else {
            OLED_GRAM[y][x + i]     = ~F8X16[c * 16 + i];
            OLED_GRAM[y + 1][x + i] = ~F8X16[c * 16 + i + 8];
        }
    } else {
        if (Is_Reverse == 0) {
            for (i = 0; i < 6; i++)
                OLED_GRAM[y][x + i] = F6x8[c][i];
        } else {
            for (i = 0; i < 6; i++)
                OLED_GRAM[y][x + i] = ~F6x8[c][i];
        }
    }
}

// 数字分解函数
unsigned int oled_pow(uint8_t m, uint8_t n)
{
    unsigned int result = 1;
    while (n--)
        result *= m;
    return result;
}

// 数字显示函数，右对齐
//  x：起点x坐标（0~127）
//  y：起点y坐标（0~3）
//  num：要显示的数值
//  len：显示位数
//  size：字体大小16/12
//  fill：填充，不足位用特定字符填充
void OLED_ShowNum(uint8_t x, uint8_t y, unsigned int num, uint8_t len, uint8_t size, char fill)
{
    uint8_t t, temp;
    uint8_t enshow = 0;
    for (t = 0; t < len; t++) {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if (enshow == 0 && t < (len - 1)) {
            if (temp == 0) {
                OLED_ShowChar(x + (size / 2) * t, y, fill, size, 0);
                continue;
            } else
                enshow = 1;
        }
        OLED_ShowChar(x + (size / 2) * t, y, temp + '0', size, 0);
    }
}

// 浮点显示函数，若设置显示小数位数为0，则仅输出整数部分并左对齐
//  x：起点x坐标（0~127）
//  y：起点y坐标（0~3）
//  num：要显示的数值
//  float_len：小数位数
void OLED_ShowFloat(uint8_t x, uint8_t y, float num, uint8_t float_len)
{
    uint8_t int_len         = 1;
    uint16_t int_num        = 1;
    uint16_t num_int_part   = 0;
    uint16_t num_float_part = 0;

    uint16_t count_int_part = 0;

    for (size_t i = 0; i < float_len; i++)
        int_num *= 10;

    num_int_part   = (uint16_t)num;
    num_float_part = (num - (float)num_int_part) * int_num;

    count_int_part = num_int_part;
    for (int_len = 0; count_int_part != 0; int_len++)
        count_int_part /= 10;

    if (num_int_part == 0) 
    {
        OLED_ShowChar(x, y, '0', 16, 0);
        if(float_len==0)
            return;
        OLED_ShowChar(x + 8, y, '.', 16, 0);
        OLED_ShowNum(x + 16, y, num_float_part, float_len, 16, '0');
    } 
    else 
    {
        OLED_ShowNum(x, y, num_int_part, int_len, 16, ' ');
        if(float_len==0)
            return;
        OLED_ShowChar(x + int_len * 8, y, '.', 16, 0);
        OLED_ShowNum(x + int_len * 8 + 8, y, num_float_part, float_len, 16, '0');
    }
}

// 显示字符串
//  x：起点x坐标（0-127）
//  y：起点y坐标（0-3）
//  *chr:字符串起始地址
//  Char_Size:选择字体 16/12
void OLED_ShowString(uint8_t x, uint8_t y, char *chr, uint8_t Char_Size)
{
    unsigned char j = 0;
    while (chr[j] != '\0') {
        OLED_ShowChar(x, y, chr[j], Char_Size, 0);
        x += 8;
        if (x > 120) {
            x = 0;
            y += 2;
        }
        j++;
    }
}

// 显示汉字
//  hzk 用取模软件得出的数组
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
    uint8_t t;
    
    for (t = 0; t < 16; t++) {
        OLED_GRAM[y][x + t] = Hzk[2 * no][t];
    }
    for (t = 0; t < 16; t++) {
        OLED_GRAM[y + 1][x + t] = Hzk[2 * no + 1][t];
    }
}
