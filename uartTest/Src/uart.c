

#include "uart.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include <stdbool.h>
#include "task.h"
#include "main.h"




void fnxUartTask(void const * argument);

osThreadId uartTaskHandle;
osSemaphoreId uartBinarySemHandle;
SemaphoreHandle_t xSemaphore;

uint8_t rxBuff[MAX_NUM_RX_BYTES];
uint8_t miniBuffRx[1];
extern UART_HandleTypeDef huart1;

void InitUartTask(void){
		
	
	xSemaphore = xSemaphoreCreateBinary();
	
    if( xSemaphore == NULL )
    {
       __NOP();
    }
	//osSemaphoreDef(uartBinarySem);
  //uartBinarySemHandle = osSemaphoreCreate(osSemaphore(uartBinarySem), 1);
	
		
	osThreadDef(uartTask, fnxUartTask, osPriorityHigh, 0, 128);
  uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);
}


eTaskState tt;
bool at = true;
void fnxUartTask(void const * argument)
{
  /* USER CODE BEGIN fnxUartTask */
  /* Infinite loop */
  for(;;)
  {
		
		
		HAL_UART_Receive_IT(&huart1,miniBuffRx, 1);

		xSemaphoreTake(xSemaphore, portMAX_DELAY);
    //osDelay(1);
  }
  /* USER CODE END fnxUartTask */
}

uint32_t countRx = 0;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	portBASE_TYPE xHighxHigherPriorityTaskWoken = pdFALSE;
	rxBuff[countRx] = miniBuffRx[0];
	countRx++;
	if (countRx>MAX_NUM_RX_BYTES){
		countRx = 0;
	}
	xSemaphoreGiveFromISR(xSemaphore, &xHighxHigherPriorityTaskWoken);
	 portEND_SWITCHING_ISR(xHighxHigherPriorityTaskWoken);
	//HAL_UART_Transmit(&huart2,rxBuff, countRx,200);
	
 //__NOP();
  
  
}


