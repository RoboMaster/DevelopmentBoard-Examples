#include "myiic.h"
#include "gpio.h"
#include "mytype.h"

#define SCL_PORT GPIOB
#define SCL_PIN GPIO_PIN_5
#define SDA_PORT GPIOB
#define SDA_PIN GPIO_PIN_4

#define IIC_SCL_0 HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET)
#define IIC_SCL_1 HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET)
#define IIC_SDA_0 HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET)
#define IIC_SDA_1 HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET)

#if 1
#define SDA_PIN_IDX 11 //PIN_IDX X = GPIO_PIN_X
#if defined(__STM32F4xx_HAL_H)
#define SDA_IN()                                      \
    {                                                 \
        SDA_PORT->MODER &= ~(3 << (SDA_PIN_IDX * 2)); \
        SDA_PORT->MODER |= (0 << (SDA_PIN_IDX * 2));  \
    } //as input

#define SDA_OUT()                                     \
    {                                                 \
        SDA_PORT->MODER &= ~(3 << (SDA_PIN_IDX * 2)); \
        SDA_PORT->MODER |= (1 << (SDA_PIN_IDX * 2));  \
    } //as output
#else
/*stm32 F1 series	only!!!*/
//only if SDA = gpio_pin_4 can use this!!!
#define SDA_IN()                     \
    {                                \
        SDA_PORT->CRL &= 0XFFF0FFFF; \
        SDA_PORT->CRL |= 8 << 16;    \
    }
#define SDA_OUT()                    \
    {                                \
        SDA_PORT->CRL &= 0XFFF0FFFF; \
        SDA_PORT->CRL |= 3 << 16;    \
    }
#endif
#else
//will slower than up!!!
static GPIO_InitTypeDef GPIO_InitStruct;
void                    SDA_OUT()
{
    GPIO_InitStruct.Pin  = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}

void SDA_IN()
{
    GPIO_InitStruct.Pin  = SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(SDA_PORT, &GPIO_InitStruct);
}
#endif

#define READ_SDA() HAL_GPIO_ReadPin(SDA_PORT, SDA_PIN)

//MPU IIC 延时函数
void IIC_Delay(void)
{
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();

    //for mpu6500 delay is very important!!!
    //for(u16 i = 0x01; i>0; i--);
}

//初始化IIC
void IIC_Init(void)
{

    //IO CONFIG IN MX_GPIO_INIT function
    //SCL  as push pull!
    //SDA		can change as INPUT or PUSH PULL
    SDA_OUT();
    IIC_SCL_1;
    IIC_SDA_1;
}
//产生IIC起始信号
void IIC_Start(void)
{
    SDA_OUT(); //sda线输出
    IIC_SDA_1;
    IIC_SCL_1;
    IIC_Delay();
    IIC_SDA_0; //START:when CLK is high,DATA change form high to low
    IIC_Delay();
    IIC_SCL_0; //钳住I2C总线，准备发送或接收数据
}
//产生IIC停止信号
void IIC_Stop(void)
{
    SDA_OUT(); //sda线输出
    IIC_SCL_0;
    IIC_SDA_0; //STOP:when CLK is high DATA change form low to high
    IIC_Delay();
    IIC_SCL_1;
    IIC_SDA_1; //发送I2C总线结束信号
    IIC_Delay();
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN(); //SDA设置为输入
    IIC_SDA_1;
    IIC_Delay(); //?
    IIC_SCL_1;
    IIC_Delay();
    while (READ_SDA()) //if sda was pull 0	means ACK ok!
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_0; //时钟输出0
    return 0;
}
//产生ACK应答
void IIC_Ack(void)
{
    IIC_SCL_0;
    SDA_OUT();
    IIC_SDA_0;
    IIC_Delay();
    IIC_SCL_1;
    IIC_Delay();
    IIC_SCL_0;
}
//不产生ACK应答
void IIC_NAck(void)
{
    IIC_SCL_0;
    SDA_OUT();
    IIC_SDA_1;
    IIC_Delay();
    IIC_SCL_1;
    IIC_Delay();
    IIC_SCL_0;
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL_0; //拉低时钟开始数据传输
    for (t = 0; t < 8; t++)
    {
        if (txd & 0x80)
            IIC_SDA_1;
        else
            IIC_SDA_0;
        txd <<= 1;
        IIC_SCL_1;
        IIC_Delay();
        IIC_SCL_0;
        IIC_Delay();
    }
}
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN(); //SDA设置为输入
    for (i = 0; i < 8; i++)
    {
        IIC_SCL_0;
        IIC_Delay();
        IIC_SCL_1;
        receive <<= 1;
        if (READ_SDA())
            receive++;
        IIC_Delay();
    }
    if (!ack)
        IIC_NAck(); //发送nACK
    else
        IIC_Ack(); //发送ACK
    return receive;
}

//return 0 if success
u8 IIC_Write_Reg(u8 devAddr, u8 reg, u8 data)
{
    IIC_Start();
    IIC_Send_Byte((devAddr << 1) | 0);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Send_Byte(data);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}
//read a specific device reg
u8 IIC_Read_Reg(u8 devAddr, u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((devAddr << 1) | 0);
    IIC_Wait_Ack();
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((devAddr << 1) | 1);
    IIC_Wait_Ack();
    res = IIC_Read_Byte(0);
    IIC_Stop();
    return res;
}

u8 IIC_Write_Bytes(u8 devAddr, u8 reg, u8* buff, u8 len)
{
    IIC_Start();
    IIC_Send_Byte((devAddr << 1) | 0);
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    for (u8 i = 0; i < len; i++)
    {
        IIC_Send_Byte(buff[i]);
        if (IIC_Wait_Ack())
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
}

u8 IIC_Read_Bytes(u8 devAddr, u8 reg, u8* buff, u8 len)
{
    IIC_Start();
    IIC_Send_Byte((devAddr << 1) | 0); //send addr + write cmd
    if (IIC_Wait_Ack())
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((devAddr << 1) | 1);
    IIC_Wait_Ack();
    while (len)
    {
        if (len == 1)
            *buff = IIC_Read_Byte(0); //send nACK
        else
            *buff = IIC_Read_Byte(1);
        len--;
        buff++;
    }
    IIC_Stop();
    return 0;
}

u8 IIC_Write(u8 addr, u8 reg, u8 len, u8* buf)
{
    return IIC_Write_Bytes(addr, reg, buf, len);
}

u8 IIC_Read(u8 addr, u8 reg, u8 len, u8* buf)
{
    //just change param index
    return IIC_Read_Bytes(addr, reg, buf, len);
}
