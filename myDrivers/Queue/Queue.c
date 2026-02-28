#include "Queue.h"
#include "main.h"

void  InitQueue(StructCirQue* pQue, DATA_TYPE* pBuf, int16_t len)
{
  int16_t i;

  pQue->front   = 0;    //队头
  pQue->rear    = 0;    //队尾
  pQue->bufLen  = len;  //缓冲区长度（队列容量）
  pQue->elemNum = 0;    //当前队列中元素的个数
  pQue->pBuffer = pBuf; //指针变量pBuf赋给指针变量pQue->pBuffer
  
  for(i = 0; i < len; i++)
  {
    pQue->pBuffer[i] = 0; //对指针变量pQue->pBuffer所指向的元素均赋初值0
  }
}

void  ClearQueue(StructCirQue* pQue)
{
  pQue->front   = 0; //队头
  pQue->rear    = 0; //队尾
  pQue->elemNum = 0; //当前队列中元素的个数
}

uint8_t    QueueEmpty(StructCirQue* pQue)
{
  return(0 == pQue->elemNum);
}

int16_t   QueueLength(StructCirQue* pQue)
{
  return(pQue->elemNum);
}

int16_t EnQueue(StructCirQue* pQue, DATA_TYPE* pInput, int16_t len)
{
  int16_t wLen = 0; //待入队的元素有len个，wLen从0增加到len-1
  
  while((pQue->elemNum < pQue->bufLen) && (wLen < len))
  {
    pQue->pBuffer[pQue->rear] = pInput[wLen]; //将待入队的第wLen个元素pInput[wLen]插入队列
    pQue->rear++;     //队尾rear自增，即指向队尾元素的下一个位置
    
    if(pQue->rear >= pQue->bufLen)
    {
      pQue->rear = 0; //如果队尾元素的下一个位置为pQue->bufLen，则rear指向队头，循环体现在此处
    }

    wLen++;
    pQue->elemNum++;  //当前队列中的元素总数
  }

  return wLen;  //如果返回值wLen为0，表示没有元素入队
}

int16_t DeQueue(StructCirQue* pQue, DATA_TYPE* pOutput, int16_t len)
{
  int16_t rLen = 0; //期望取出len个元素，最终能取出rLen个元素
  
  while((pQue->elemNum > 0) && (rLen < len))
  {
    pOutput[rLen] = pQue->pBuffer[pQue->front];
    pQue->front++;
    
    if( pQue->front >= pQue->bufLen )
    {
      pQue->front = 0;//如果队头元素的下一个位置为pQue->bufLen，则front指向队头，循环体现在此处
    }

    rLen++;
    pQue->elemNum--;  //当前队列中的元素总数
  }

  return rLen;  //如果返回值rLen为0，表示队列中没有元素
}
