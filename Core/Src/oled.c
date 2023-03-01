#include "oled.h"
#include "oledfont.h"

extern I2C_HandleTypeDef hi2c1;     // ����I2CͨѶ��
#define PAGE 4                      // ����ҳ����128*64Ϊ8ҳ��128*32Ϊ4ҳ
uint8_t OLED_GRAM[PAGE][128] = {0}; // ͼ�񻺴�ҳ��x128��

// ��ʼ������
uint8_t CMD_Start[] = {
    // 0.91����������
    0xAE,       // �ر���ʾ
    0xA4,       // ��ʾģʽ(������ʾ:0xA4 ǿ��ȫ��:0xA5)
    0xA6,       // ������ʾ��ʽ(������ʾ:0xA6 ������ʾ:0xA7)
    0x81, 0xC8, // �Աȶ�����(���ȵ���) (1~255��Ӧ0x00~0xff)
    0x8D, 0x14, // ��������(����:0x14 ����:0x10)
    0xA1, 0xC8, // ���ҷ��ù�(����ӳ��),���·��ù�(����ӳ��)
    0xA8, 0x1F, // ����MUX��(��ʾ����) (64��:0x3F 32��:0x1F)
    0xD3, 0x00, // ���ô�ֱ��ʾ����ƫ�ƣ�Ĭ��ֵ00 û��ƫ��
    0xDA, 0x02, // ����COMӲ����������,��Ӧ�ֱ���(64��:0x12 32��:0x02)
};
// Ѱַģʽ��������
uint8_t CMD_Frame[] = {

    // ˮƽ/��ֱר��
    0x20, 0x00,       // �����ڴ��ַģʽ��ˮƽ0x00����ֱ0x01
    0x21, 0x00, 0x7f, // �����е�ַ��0~127
    0x22, 0x00, 0x03, // ����ҳ��ַ��0��ָ����Χ(64��:0x07 32��:0x03)

    // ҳ��ַѰַר��
    // 0x20, 0x02,    // �����ڴ��ַģʽ��ҳѰַ0x02
    // 0x00, 0x10,    // �����е�ַ��0��ʼ
    // 0xB0,          // ����ҳ��ַ��0��ʼ(0-7��Ӧ0xB0-0xB7)

    0xAF // ������ʾ
};

// ���ͳ�ʼ������
void OLED_Start(void)
{
    // ͨ��I2C���ͣ�������0x00��ʾ�������0x40��ʾ������ʾ����
    HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Start, sizeof(CMD_Start));
    HAL_Delay(10);
    HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x00, I2C_MEMADD_SIZE_8BIT, CMD_Frame, sizeof(CMD_Frame));
}

// ������ҳ����
void OLED_Refreash(void)
{
    // ѭ������ÿ������
    for (size_t i = 0; i < PAGE; i++) {
        // �ȴ�i2c׼������
        for (size_t j = 0; j < 10000 && HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY; j++);
        // ����һ��������
        HAL_I2C_Mem_Write_DMA(&hi2c1, 0x78, 0x40, I2C_MEMADD_SIZE_8BIT, OLED_GRAM[i], 128);
    }
}

// ��ʼ��oled��Ļ����
void OLED_Init(void)
{
    HAL_Delay(300); // �������ʱ����Ҫ
    OLED_Start();
    OLED_Clear();
    OLED_Refreash();
}

// ����
void OLED_Clear(void)
{
    for (size_t i = 0; i < 4; i++) {
        for (size_t j = 0; j < 128; j++) {
            OLED_GRAM[i][j] = 0x00;
        }
    }
}

// ����
//  x:0~127
//  y:0~31
//  t:1 ��� 0,���
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

// ����
//  x1,y1:�������
//  x2,y2:��������
//  mode:1 ��� 0,���
void OLED_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t mode)
{
    uint16_t t;
    int xerr = 0, yerr = 0, delta_x, delta_y, distance;
    int incx, incy, uRow, uCol;
    delta_x = x2 - x1; // ������������
    delta_y = y2 - y1;
    uRow    = x1; // �����������
    uCol    = y1;
    if (delta_x > 0)
        incx = 1; // ���õ�������
    else if (delta_x == 0)
        incx = 0; // ��ֱ��
    else {
        incx    = -1;
        delta_x = -delta_x;
    }
    if (delta_y > 0)
        incy = 1;
    else if (delta_y == 0)
        incy = 0; // ˮƽ��
    else {
        incy    = -1;
        delta_y = -delta_x;
    }
    if (delta_x > delta_y)
        distance = delta_x; // ѡȡ��������������
    else
        distance = delta_y;
    for (t = 0; t < distance + 1; t++) {
        OLED_DrawPoint(uRow, uCol, mode); // ����
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

// ��ˮƽ����(�������꣬����Ϊ���㣬������)
//  XS      ����ʼ����(0~127)
//  XE      ����ֹ����(0~127)
//  Y       ������(0~31)
//  Len     ���ߵ����߿��
void OLED_DrawDashline(uint8_t XS, uint8_t XE, uint8_t Y, uint8_t Len)
{
    for (size_t i = XS; i <= XE; i += 8)
        OLED_DrawLine(i, Y, i + Len, Y, 1);
}

// x��Բ��x���꣨0~127��
// y��Բ��y���꣨0~31��
// r:Բ�İ뾶
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
        num = (a * a + b * b) - r * r; // ���㻭�ĵ���Բ�ĵľ���
        if (num > 0) {
            b--;
            a--;
        }
    }
}

// ��ָ��λ����ʾһ���ַ�,���������ַ�
//  x�����x���꣨0~127��
//  y�����y���꣨0~3��
//  chr:�ַ�
//  Char_Size:ѡ������ 16/12
//  Is_Reverse:0������ʾ,1������ʾ
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t Char_Size, uint8_t Is_Reverse)
{
    unsigned char c = 0, i = 0;
    c = chr - ' '; // �õ�ƫ�ƺ��ֵ
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

// ���ַֽ⺯��
unsigned int oled_pow(uint8_t m, uint8_t n)
{
    unsigned int result = 1;
    while (n--)
        result *= m;
    return result;
}

// ������ʾ�������Ҷ���
//  x�����x���꣨0~127��
//  y�����y���꣨0~3��
//  num��Ҫ��ʾ����ֵ
//  len����ʾλ��
//  size�������С16/12
//  fill����䣬����λ���ض��ַ����
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

// ������ʾ��������������ʾС��λ��Ϊ0���������������ֲ������
//  x�����x���꣨0~127��
//  y�����y���꣨0~3��
//  num��Ҫ��ʾ����ֵ
//  float_len��С��λ��
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

// ��ʾ�ַ���
//  x�����x���꣨0-127��
//  y�����y���꣨0-3��
//  *chr:�ַ�����ʼ��ַ
//  Char_Size:ѡ������ 16/12
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

// ��ʾ����
//  hzk ��ȡģ����ó�������
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
