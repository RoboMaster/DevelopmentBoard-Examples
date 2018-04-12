
#include "mytype.h"
void IIC_Delay(void);
//初始化IIC
void IIC_Init(void);
//产生IIC起始信号
void IIC_Start(void);
//产生IIC停止信号
void IIC_Stop(void);
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 IIC_Wait_Ack(void);
//产生ACK应答
void IIC_Ack(void);
//不产生ACK应答
void IIC_NAck(void);
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答
void IIC_Send_Byte(u8 txd);
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
u8 IIC_Read_Byte(unsigned char ack);

void IIC_Init(void);
//iic读写寄存器
u8 IIC_Write_Reg(u8 devAddr, u8 reg, u8 data);
u8 IIC_Read_Reg(u8 devAddr, u8 reg);

//从制定的器件寄存器地址连续读len个字节的数据。相当于reg， reg+1， 。。。，reg+len
u8 IIC_Write_Bytes(u8 devAddr, u8 reg, u8* buff, u8 len);
u8 IIC_Read_Bytes(u8 devAddr, u8 reg, u8* buff, u8 len);

//for inv_dmp.c call use
u8 IIC_Read(u8 addr, u8 reg, u8 len, u8* buf);
u8 IIC_Write(u8 addr, u8 reg, u8 len, u8* buf);
