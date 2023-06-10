// STANDARD INCLUDES
#include <stdio.h>l.g 
#include <conio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// KERNEL INCLUDES
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

// HARDWARE SIMULATOR UTILITY FUNCTIONS  
#include "HW_access.h"


// SERIAL SIMULATOR CHANNEL TO USE 
#define COM_CH_0 (0)
#define COM_CH_1 (1)


// TASK PRIORITIES 
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 2 )
#define TASK_SERIAl_REC_PRI			( tskIDLE_PRIORITY + 3 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + 1 )
//#define	OBRADA_TASK_PRI				( tskIDLE_PRIORITY + 1 )


// TASKS: FORWARD DECLARATIONS 

//void SerialSend_Task(void* pvParameters);
void SerialReceive_Task0(void* pvParameters);
void Obrada_vrata_Task(void* pvParameters);
void Obrada_brzine_Task(void* pvParameters);
void Led_Displej_Task(void* pvParameters);
void Alarm_Task(void* pvParameters);
 void SerialSend_Task(void* pvParameters);


// TRASNMISSION DATA - CONSTANT IN THIS APPLICATION 




// RECEPTION DATA BUFFER - COM 0
#define R_BUF_SIZE (32)
uint8_t r_buffer[R_BUF_SIZE];
unsigned volatile r_point;

#define P_BUF_SIZE (32)
uint8_t p_buffer[P_BUF_SIZE];
unsigned volatile p_point;

#define T_BUF_SIZE (32)
uint8_t t_buffer[T_BUF_SIZE];
unsigned volatile t_point;

static char trigger[] = "Upozorenje otvorena su vrata 1 ";
static char trigger1[] = "Upozorenje otvorena su vrata 2";
static char trigger2[] = "Upozorenje otvorena su vrata 3";
static char trigger3[] = "Upozorenje otvorena su vrata 4";
static char trigger4[] = "Upozorenje otvorena su vrata 5";

unsigned volatile n_point;

// 7-SEG NUMBER DATABASE - ALL HEX DIGITS [ 0 1 2 3 4 5 6 7 8 9 A B C D E F ]
static const char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };


typedef struct  _Mystruct
{
	uint8_t stanje_vrata;
	uint8_t  vrata;
}Mystruct;


static QueueHandle_t Serial_Queue0;
static QueueHandle_t Serial_Queue1;
static QueueHandle_t Serial_Queue2;
static QueueHandle_t Serial_Queue3;
static QueueHandle_t Serial_Queue4;
static QueueHandle_t Serial_Queue5;

SemaphoreHandle_t Send_Semaphore;
SemaphoreHandle_t RXC_BinarySemaphore;
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t RXC1_BinarySemaphore;
//SemaphoreHandle_t TBE_BinarySemaphore;



static uint32_t prvProcessRXCInterrupt(void) {	// RXC - RECEPTION COMPLETE - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0)
		xSemaphoreGiveFromISR(RXC_BinarySemaphore, &xHigherPTW);
	if (get_RXC_status(1) != 0)
		xSemaphoreGiveFromISR(RXC1_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}

static uint32_t OnLED_ChangeInterrupt(void) {	// OPC - ON INPUT CHANGE - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

/*
static uint32_t prvProcessTBEInterrupt(void) {	// TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER 
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(TBE_BinarySemaphore, &xHigherPTW);
	portYIELD_FROM_ISR(xHigherPTW);
}

*/

// MAIN - SYSTEM STARTUP POINT 
void main_demo(void) {
	// INITIALIZATION OF THE PERIPHERALS
	init_7seg_comm();
	init_LED_comm();
	init_serial_uplink(COM_CH_1);		// inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(COM_CH_0);	// inicijalizacija serijske RX na kanalu 0
	init_serial_downlink(COM_CH_1);


	// INTERRUPT HANDLERS
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);		// ON INPUT CHANGE INTERRUPT HANDLER 
	//vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);	// SERIAL TRANSMITT INTERRUPT HANDLER 
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);	// SERIAL RECEPTION INTERRUPT HANDLER 

	// BINARY SEMAPHORES
	Send_Semaphore = xSemaphoreCreateBinary();
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();// CREATE LED INTERRUPT SEMAPHORE 
	//TBE_BinarySemaphore = xSemaphoreCreateBinary();		// CREATE TBE SEMAPHORE - SERIAL TRANSMIT COMM 
	RXC_BinarySemaphore = xSemaphoreCreateBinary();	
	//RXC1_BinarySemaphore = xSemaphoreCreateBinary();		// CREATE RXC SEMAPHORE - SERIAL RECEIVE COMM

	//QUEUES
	Serial_Queue0 = xQueueCreate(2, sizeof(Mystruct));
	Serial_Queue1 = xQueueCreate(2, sizeof(uint8_t));
	Serial_Queue2 = xQueueCreate(2, sizeof(Mystruct));
	Serial_Queue3 = xQueueCreate(2, sizeof(Mystruct));
	Serial_Queue4 = xQueueCreate(2, sizeof(Mystruct));
	Serial_Queue5 = xQueueCreate(2, sizeof(Mystruct));
//	Serial_Queue6 = xQueueCreate(2, sizeof(uint8_t));
	// TASKS 

	BaseType_t status;
	status = xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);	// SERIAL TRANSMITTER TASK 
	status = xTaskCreate(SerialReceive_Task0, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);
	status = xTaskCreate(Obrada_vrata_Task, NULL, configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	status = xTaskCreate(Obrada_brzine_Task, NULL, configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	status = xTaskCreate(Led_Displej_Task, NULL, configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);	
	status = xTaskCreate(Alarm_Task, NULL, configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
   // status = xTaskCreate(SerialReceive_Task1, "SRx1", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAl_REC_PRI, NULL);// SERIAL RECEIVER TASK 
	r_point = 0;
	p_point = 0;
	t_point = 0;


	// START SCHEDULER
	vTaskStartScheduler();
	while (1);
}





void SerialReceive_Task0(void* pvParameters)
{
	uint8_t cc = 0;
	uint8_t pp = 0;
	Mystruct rec0_s;
	while (1) 
	{
		xSemaphoreTake(RXC_BinarySemaphore, portMAX_DELAY);// ceka na serijski prijemni interapt
		get_serial_character(COM_CH_0, &cc);
		get_serial_character(COM_CH_0, &pp);
		
		//printf("KANAL 0: primio karakter: %u\n", (unsigned)cc);// prikazuje primljeni karakter u cmd prompt

		if (cc == 0xef) {	// EF oznacava POCETAK poruke
			r_point = 0;
		}
		else if (cc == 0xff) {
			rec0_s.stanje_vrata = r_buffer[0];
			rec0_s.vrata = r_buffer[1];
			//printf("stanje vrat : %d, vrata : %d\n", rec0_s.stanje_vrata, rec0_s.vrata);
			xQueueSend(Serial_Queue0, &rec0_s, 0);
		}
		else if (r_point < R_BUF_SIZE) { // pamti karaktere izmedju EF i FF
			r_buffer[r_point++] = cc;
		}


		if (pp == 0xfe)
		{
			p_point = 0;
		}
		else if (pp == 0xee)
		{
			
			xQueueSend(Serial_Queue1, &p_buffer, 0);
		}
		else if (p_point < P_BUF_SIZE)
		{ 
			p_buffer[p_point++] = pp;
		}


	}
}


void Obrada_vrata_Task(void* pvParameters)
{
	Mystruct obrada_v;

	while (1)
	{
		xQueueReceive(Serial_Queue0, &obrada_v, portMAX_DELAY);


		

		if (obrada_v.stanje_vrata == 0x00)
		{
			printf("Vrata automobila zatvorena\n");
		}
		else if (obrada_v.stanje_vrata == 0x01)
		{
			printf("Vrata automobila otvorena\n");

		}
		else
		{
			printf("GRESKA , pogresan unos\n");
		}

		if (obrada_v.vrata == 0x01)
		{
			printf("Prednja leva vrata\n");

		}
		else if (obrada_v.vrata == 0x02)
		{
			printf("Prednja desna vrata\n");

		}
		else if (obrada_v.vrata == 0x03)
		{
			printf("Zadnja leva vrata\n");

		}
		else if (obrada_v.vrata == 0x04)
		{
			printf("Zadnja desna vrata\n");


		}
		else if (obrada_v.vrata == 0x05)
		{
			printf("Gepek\n");

		}
		else
		{
			printf("GRESKA:pogreasn unos \n");
		}

	xQueueSend(Serial_Queue2, &obrada_v, 0);


	}
	
}

void Obrada_brzine_Task(void* pvParameters)
{
	uint8_t jedinice;
	uint8_t desetice;
	uint8_t stotine;
	uint8_t max_brzina = 5;
	uint8_t trenutna_brzina;
	Mystruct obrada_b;

	while(1)
	{

		xQueueReceive(Serial_Queue1, &p_buffer, portMAX_DELAY);
		

		jedinice = p_buffer[2];
		desetice = p_buffer[1];
		stotine = p_buffer[0];
		trenutna_brzina = (100 * stotine) + (10 * desetice) + jedinice;
		printf("ispisi brzinu: %d\n", trenutna_brzina);


		xQueueReceive(Serial_Queue2, &obrada_b, portMAX_DELAY);
		

		if (trenutna_brzina > max_brzina && obrada_b.stanje_vrata == 0x01)

		{
			printf("vrata1 su %d\n", obrada_b.vrata);
			xSemaphoreGive(Send_Semaphore, 0);
			//xQueueSend(Serial_Queue5, &obrada_b, 0);

			xQueueSend(Serial_Queue3, &obrada_b, 0);
			xQueueSend(Serial_Queue4, &obrada_b, 0);
			xQueueSend(Serial_Queue5, &obrada_b, 0);
		}
		else
		{
			printf("Trenutna brzina je manja od maksimalne");
		}

	}

}
void Led_Displej_Task(void* pvParameters)
{
	Mystruct led_s;
	uint8_t i;

	while (1)
	{
		xQueueReceive(Serial_Queue3, &led_s, portMAX_DELAY);

		 printf("vrata su %d\n", led_s.vrata);

		if (led_s.vrata != 0x05)
		{

			select_7seg_digit(0);
			set_7seg_digit(0x5E);
			select_7seg_digit(1);
			set_7seg_digit(0x5C);
			select_7seg_digit(2);
			set_7seg_digit(0x5C);
			select_7seg_digit(3);
			set_7seg_digit(0x50);
			//printf("vrata : %d", led_s.vrata);

			if (led_s.vrata == 0x01)
			{
				select_7seg_digit(4);
				set_7seg_digit(0x06);
			}
			else if (led_s.vrata == 0x02)
			{
				select_7seg_digit(4);
				set_7seg_digit(0x5B);
			}
			else if (led_s.vrata == 0x03)
			{
				select_7seg_digit(4);
				set_7seg_digit(0x4F);
			}
			else if (led_s.vrata == 0x04)
			{
				select_7seg_digit(4);
				set_7seg_digit(0x66);
			}




			for (i = 0; i < 5; i++)
			{

				set_LED_BAR(1, 0xff);
				set_LED_BAR(2, 0xff);
				set_LED_BAR(3, 0xff);
				vTaskDelay(pdMS_TO_TICKS(500));
				set_LED_BAR(1, 0x00);
				set_LED_BAR(2, 0x00);
				set_LED_BAR(3, 0x00);
				vTaskDelay(pdMS_TO_TICKS(500));
			}



		}
	}

}

void Alarm_Task(void* pvParameters)
{
	Mystruct alarm_s;
	 static uint8_t tmp;
	 uint8_t alarm;
	 

	xQueueReceive(Serial_Queue4, &alarm_s, portMAX_DELAY);
	//xQueueReceive(Serial_Queue6, &t_buffer, portMAX_DELAY);

	if (alarm_s.vrata == 0x05)
	{
		


		while (1)

		{



				xQueueSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY);

				get_LED_BAR(0, &tmp);

				if ((tmp & 0x01) != 0)
				{
					alarm = 1;
					
					
					
					set_LED_BAR(2, 0xff);
					set_LED_BAR(3, 0x00);
					set_LED_BAR(1, 0x00);
						

					select_7seg_digit(0);
					set_7seg_digit(0x5E);
					select_7seg_digit(1);
					set_7seg_digit(0x5C);
					select_7seg_digit(2);
					set_7seg_digit(0x5C);
					select_7seg_digit(3);
					set_7seg_digit(0x50);
					select_7seg_digit(4);
					set_7seg_digit(0x3F);



				}

				else 
				{
					alarm = 0;
					set_LED_BAR(3, 0xff);
					set_LED_BAR(2, 0x00);
					set_LED_BAR(1, 0x00);
					
					select_7seg_digit(0);
					set_7seg_digit(0x5E);
					select_7seg_digit(1);
					set_7seg_digit(0x5C);
					select_7seg_digit(2);
					set_7seg_digit(0x5C);
					select_7seg_digit(3);
					set_7seg_digit(0x50);
					select_7seg_digit(4);
					set_7seg_digit(0x6D);

				}
				

				




			
		}


	}
}

		/*
void SerialReceive_Task1(void* pvParameters)
{

	char tt = 0;
	uint8_t naredba;
	


	while (1)
	{
		xSemaphoreTake(RXC1_BinarySemaphore, portMAX_DELAY);// ceka na serijski prijemni interapt
		get_serial_character(COM_CH_1, &tt);


		printf("KANAL 0: primio karakter: %c\n",tt);// prikazuje primljeni karakter u cmd prompt


		if (tt == '\0d') {
			//memcpy(poruka, t_buffer, 100);
			printf("ispis %c \n", t_buffer[0]);
			naredba = t_buffer[0];
			if (naredba == '0')
			{
				//vTaskDelete(Alarm_Task);
			}
			
		}
		else if (t_point < T_BUF_SIZE) { // pamti karaktere izmedju EF i FF
			t_buffer[t_point++] = tt;
		}
	}


}
*/

void SerialSend_Task(void* pvParameters)
{
	Mystruct send_s;
	
	
	

	uint8_t n_point = 0;
	
	xSemaphoreTake(Send_Semaphore, portMAX_DELAY);
	xQueueReceive(Serial_Queue5, &send_s, portMAX_DELAY);
	printf("ispsi vrat: %d\n", send_s.vrata);

	while (1) 
	{   
		
		
		if (send_s.vrata == 0x01)
		{
			if (n_point > (sizeof(trigger) - 1))
				n_point = 0;
			send_serial_character(COM_CH_1, trigger[n_point++]);

		}
		
		
		else if (send_s.vrata == 0x02)
		{
			if (n_point > (sizeof(trigger1) - 1))
				n_point = 0;
			send_serial_character(COM_CH_1, trigger1[n_point++]);

		}
		else if (send_s.vrata == 0x03)
		{
			if (n_point > (sizeof(trigger2) - 1))
				n_point = 0;
			send_serial_character(COM_CH_1, trigger2[n_point++]);

		}
		else if (send_s.vrata == 0x04)
		{
			if (n_point > (sizeof(trigger3) - 1))
				n_point = 0;
			send_serial_character(COM_CH_1, trigger3[n_point++]);

		}
		else if (send_s.vrata == 0x05)
		{
			if (n_point > (sizeof(trigger4) - 1))
				n_point = 0;
			send_serial_character(COM_CH_1, trigger4[n_point++]);

		}

		vTaskDelay(pdMS_TO_TICKS(200));// kada se koristi vremenski delay
		
	}
	


}