/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdio.h>		//biblioteka wejscia/wyjscia
#include <stdint.h>		//biblioteka standardowa int
#include <string.h>		//biblioteka do obslugi ciagow znakow string
#include <math.h>		//biblioteka z funkcjami matematycznymi (exp itp.)
#include "stm32f10x.h"	//biblioteka mikrokontrolera stm32f103
#include "servo.h"		//mini-biblioteka z konfiguracja serwa (skrajne pozycje, funkcja ustawiania pozycji itd.)
#include "send.h"		//mini-biblioteka z funkcjami wysylajacymi dane przez USART do PC
#include "fsr.h"		//mini-biblioteka z funkcja konwertujaca napiecie na czujniku sily na sile

#define X_MIN 0			//minimalne rozwarcie szczek w [mm]
#define X_MAX 55		//maksymalne rozwarcie szczek w [mm]
#define T_SEND 0.05		//czas w [s] co jaki sa wysylane dane do PC

#define ADC_CHANNELS 4										//ilosc wykorzystywanych ADC
uint16_t adc_value[ADC_CHANNELS];							//tablica na wartosci sczytywane z ADC poprzez DMA
uint16_t v_value[ADC_CHANNELS];								//tablica na wartosci napiecia w [mV] z ADC
float Fm = 0, Xm = X_MAX, Vm = 0, Im = 0, Xm0 = X_MAX;		//zmienne na wartosci aktualne (przeliczone z czujnikow)

uint8_t start = 0;						//zmienna startu (1-start, 0-stop)
int tryb = 0;							//tryb regulacji (1-pozycja, 2-admitancja, ...) (sczytywany z aplikacji z PC)
float Fd = 0, Xd = X_MAX, Vd = 0;		//wartosci zadane (sczytywane z aplikacji z PC)
float M = 0, B = 0, K = 0, T = 0.1;		//wspolczynniki i czas probkowania [s] (sczytywane z aplikacji z PC)

float Xt = X_MAX, Vt = 0;				//wartosci nastawy w aktualnej probce (liczone wg wybranego regulatora)
float Xpom1 = 0, Xpom2 = 0, Xpom3 = 0;	//zmienne pomocnicze (do liczenia calek i sum)
float W1 = 1;							//wagi wartosci z czujnikow (zsumowane s¹ równe 1)
float W2 = 0;							//dobrane zosta³y jako 10x czas probkowania T (daje to wygladzenie wartosci bez zauwazalnego opoznienia)

char komenda[100], c[2];				//tablica na komende sterujaca z aplikacji i miejsce na kolejne znaki tej komendy
char stan[50] = "OK";					//miejsce na stan regulacji/komunikat o bledzie


void set_gripper(uint8_t mode)	//funkcja ustawienia parametrow przy starcie/stopie chwytaka
{
	start = 0;											//na czas ustawiania wartosci STOP

	TIM_TimeBaseInitTypeDef tim;						//zmienna tim do modyfikacji czasu probkowania timera
	TIM_TimeBaseStructInit(&tim);						//
	tim.TIM_CounterMode = TIM_CounterMode_Up;			//zliczanie w gore (od 0 do period)
	tim.TIM_Prescaler = 64000 - 1;						//preskaler

	if(mode) {											//jesli START to:
		tim.TIM_Period = (uint16_t)(1000*T_SEND - 1);	//czestsze wysylanie danych
		TIM_TimeBaseInit(TIM3, &tim);					//inicjalizacja timera
		GPIO_SetBits(GPIOA, GPIO_Pin_5);				//zapala diode gdy start
	}
	else {												//jesli STOP to:
		tryb = 0;										//przywracanie domyslnych nastaw
		Fd = 0, Xd = X_MAX, Vd = 0;						//...
		M = 0, B = 0, K = 0, T = 0.1;					//...
		tim.TIM_Period = (uint16_t)(1000*0.1 - 1);		//rzadsze wysylanie danych
		TIM_TimeBaseInit(TIM3, &tim);					//inicjalizacja timera
		GPIO_ResetBits(GPIOA, GPIO_Pin_5);				//gasi diode gdy stop
	}

	Xt = X_MAX, Vt = 0;									//przywracanie domyslnych wartosci zmiennych
	Xpom1 = 0, Xpom2 = 0, Xpom3 = 0;					//...
	W1 = T * 10;										//...
	W2 = 1 - W1;										//...

	tim.TIM_Period = (uint16_t)(1000*T - 1);			//nowy czas probkowania T
	TIM_TimeBaseInit(TIM2, &tim);						//inicjalizacja timera

	if(mode) start = 1;									//START REGULACJI
}

//--- obsluga przerwan
void USART2_IRQHandler()		//obsluga komend wysylanych z PC do mikrokontrolera
{								//format komendy: "<tryb sila pozycja predkosc inercja tlumienie sztywnosc czas_probkowania>"
	if (USART_GetITStatus(USART2, USART_IT_RXNE)) {
		if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE)) {
			c[0] = USART_ReceiveData(USART2); //zapisanie otrzymanego chara
			switch (c[0]) {
				case '<': //'<' oznacza poczatek komendy - czysci string i zaczyna komende na nowo
					komenda[0] = '<';
					komenda[1] = '\0';
					break;
				case '>': //'>' oznacza koniec komendy - sprawdza poprawnosc i ja wykonuje
					strcat(komenda, c);
					if(komenda[0] == '<' && komenda[strlen(komenda) - 1] == '>' && strlen(komenda) > 40 && strlen(komenda) < 70) {
						if(sscanf(komenda, "<%d %f %f %f %f %f %f %f>", &tryb, &Fd, &Xd, &Vd, &M, &B, &K, &T) == 8) {
							set_gripper(1); //jesli komenda jest poprawna to startuje
							strcpy(stan, "OK - start");
						}
						else {
							set_gripper(0); //jesli niepoprawna to zatrzymuje dzialanie ukladu
							strcpy(stan, "Blad przy odczytywaniu komendy");
						}
					}
					else {
						set_gripper(0); //zatrzymanie
						strcpy(stan, "Bledny format komendy");
					}
					komenda[0] = '\0';
					break;
				case 's': //komenda 's' oznacza stop
					set_gripper(0); //zatrzymanie
					strcpy(stan, "OK - stop");
					break;
				default:
					strcat(komenda, c); //dodanie kolejnego chara (znaku) do istniejacej komendy
					break;
			}
			if(strlen(komenda) > 70) //zabezpieczenie przed 'zapelnieniem' zmiennej
				komenda[0] = '\0';
		}
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}
}

void TIM3_IRQHandler()			//timer uzywany do wysylania danych do PC (sily, pozycji, predkosci i pradu w serwie)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
        printf("%.0f %.1f %.1f %.0f | %s\n", Fm, Xm, Vm, Im, stan); //wysylanie danych do PC
    }
}

void TIM2_IRQHandler()			//glowny timer wykonujacy sie co czas probkowania T (liczenie kolejnych nastaw i sterowanie)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update)) {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        v_value[0] = adc_value[0] * 3300 / 4096; //zamiana na [mV] wartosci z ADC[0:4095] czujnik nacisku 1
        v_value[1] = adc_value[1] * 3300 / 4096; //zamiana na [mV] wartosci z ADC[0:4095] czujnik nacisku 2
        v_value[2] = adc_value[2] * 3300 / 4096; //zamiana na [mV] wartosci z ADC[0:4095] pozycja serwa
        v_value[3] = adc_value[3] * 3300 / 4096; //zamiana na [mV] wartosci z ADC[0:4095] prad serwa

        Fm = W1*((volt2force(v_value[0]) + volt2force(v_value[1])) / 2) + W2*Fm;	//wartosc aktualna sily (srednia z 2 czujnikow) w [g]
        Xm = W1*(X_MAX*cos(M_PI*(0.6558 * v_value[2] - 169.4288)/3600)) + W2*Xm;	//wartosc aktualna odleglosci szczek w [mm]
        Vm = W1*((Xm - Xm0) / T) + W2*Vm;											//wartosc aktualna predkosci szczek w [mm/s] (pochodna pozycji)
        Im = W1*((v_value[3] - 318) * 1.25) + W2*Im;								//wartosc aktualna pradu w serwie w [mA]
        Xm0 = Xm;																	//zapamietanie poprzedniej pozycji w celu wyliczenia pochodnej

        if(Fm > 2000) { //ograniczenie sily maksymalnej
        	set_gripper(0);
        	strcpy(stan, "STOP - przekroczono sile nacisku");
        }
        if(Im > 2000) { //ograniczenie pradu maksymalnego
        	set_gripper(0);
        	strcpy(stan, "STOP - przekroczono pobor pradu");
        }

        if(start) { //gdy start = 1 to wybiera odpowiedni regulator i rozpoczyna regulacje
			switch(tryb) {
			//========================================================================================================//
				case 1: //regulator pozycyjny
					/*
					Xpom3 = (Xd - Xm - Xpom1) / T; //pochodna uchybu
					Xpom1 = Xd - Xm; //uchyb pozycji
					Xpom2 += Xpom1 * T; //calka z uchybu
					Vt = 1.6*Xpom1 + 0.001*Xpom2 + 0.1*Xpom3; //liczenie predkosci regulatorem PID
					if(Vt > X_MAX) Vt = X_MAX; //ograniczenie predkosci maksymalnej
					else if(Vt < -X_MAX) Vt = -X_MAX; //ograniczenie predkosci minimalnej
					Xt = Xt + Vt * T; //calkowanie predkosci w celu otrzymania pozycji
					*/
					Xt = Xd; //nastawa pozycji = pozycja zadana
					break;
			//========================================================================================================//
				case 2: //regulator admitancyjny
					Xpom1 += Vt; //suma wszystkich poprzednich predkosci
					Vt = (Fd-Fm - K*T*Xpom1 + M/T*Vt) / (M/T + K*T + B); //liczenie predkosci
					Xt += Vt * T; //calkowanie predkosci w celu otrzymania pozycji
					break;
			//========================================================================================================//
				case 3: //regulator impedancyjny
					Xpom3 += Xpom2; //suma wszystkich poprzednich predkosci w sprzezeniu zwrotnym
					Xpom2 = (Fm - K*T*Xpom3 + M/T*Xpom2) / (M/T + K*T + B); //obliczenie w sprzezeniu zwrotnym predkosci z sily
					Xpom1 += Xpom2 * T; //calkowanie otrzymanej w sprzezeniu zwrotnym predkosci aby otrzymac pozycje
					Xt = Xd - Xpom1; //liczenie nastawy pozycji
					break;
			//========================================================================================================//
				case 4: //regulator sztywnosci
					Xt = Xd - Fm / K; //liczenie pozycji
					break;
			//========================================================================================================//
				case 5: //regulator tlumienia
					Vt = Vd - Fm / B; //liczenie predkosci
					Xt += Vt * T; //calkowanie predkosci w celu otrzymania pozycji
					break;
			//========================================================================================================//
				case 6: //rownolegly regulator pozycyjno-silowy
					Xpom1 += Fm * T; //calka z sily Fm
					Vt = Vd - Xpom1*K - Fm/B; //liczenie predkosci (K odpowiada wspolczynnikowi calkowania C)
					Xt += Vt * T; //calkowanie predkosci w celu otrzymania pozycji
					break;
			//========================================================================================================//
				default:
					set_gripper(0);
					strcpy(stan, "STOP - Bledny tryb sterowania");
					break;
			//========================================================================================================//
			}

			if(Xt > X_MAX) Xt = X_MAX;		//ograniczenie pozycji maksymalnej
			else if(Xt < X_MIN)	Xt = X_MIN;	//ograniczenie pozycji minimalnej

			set_position((int)(acos(Xt/X_MAX)*3600/M_PI), 0); //ustawienie pozycji w postaci [0:1800]
        }
        else {
        	set_position(0, 0); //ustawienie pozycji maksymalnej (rozwarcie szczek chwytaka)
        }
    }
}
//--- koniec obslugi przerwan

int main(void)					//glowna funkcja inicjalizujaca wszystkie podzespoly mikrokontrolera (porty, komunikacje USART, obsluge przerwan, ADC, timery, PWM i DMA)
{
	GPIO_InitTypeDef gpio;
	USART_InitTypeDef uart;
	NVIC_InitTypeDef nvic;
	ADC_InitTypeDef adc;
	TIM_TimeBaseInitTypeDef tim;
	TIM_OCInitTypeDef  channel;
	DMA_InitTypeDef dma;

	//podlaczenie zegarow do poszczegolnych ukladow
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); // zmniejszamy preskalerem czestowtliwosc adc na 64/6 MHz
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	//konfiguracja PINOW GPIO
	GPIO_StructInit(&gpio);
	gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	gpio.GPIO_Mode = GPIO_Mode_AIN; //pin PA0, PA1, PA4 jako analogowe wejscie (do ADC)
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_0;
	gpio.GPIO_Mode = GPIO_Mode_AIN; //pin PB0 jako analogowe wejscie (do ADC)
	GPIO_Init(GPIOB, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_2;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP; //pin PA2 jako wyjscie USART
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING; //pin PA3 jako wejscie USART
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_5;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP; //dioda na plytce (PA5) jako wyjscie
	GPIO_Init(GPIOA, &gpio);

	gpio.GPIO_Pin = GPIO_Pin_8;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Mode = GPIO_Mode_AF_PP; //pin PB8 jako wyjscie na PWM do serwa
	GPIO_Init(GPIOB, &gpio);

	//komunikacja z PC za pomoca USART
	USART_StructInit(&uart);
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	uart.USART_BaudRate = 115200;
	USART_Init(USART2, &uart);
	USART_Cmd(USART2, ENABLE);
	//kontroler przerwan USART
	nvic.NVIC_IRQChannel = USART2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;
	nvic.NVIC_IRQChannelSubPriority = 0x00;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	//dma w celu odciazenia procesora od ADC
	DMA_StructInit(&dma);
	dma.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; //adres zrodlowy
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //brak inkrementacji wskaznika
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //rozmiar danych
	dma.DMA_MemoryBaseAddr = (uint32_t)adc_value; //adres docelowy
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable; //inkrementacja wskaznika
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //rozmiar danych
	dma.DMA_DIR = DMA_DIR_PeripheralSRC;
	dma.DMA_BufferSize = ADC_CHANNELS; //rozmiar bufora
	dma.DMA_Mode = DMA_Mode_Circular; //po zapelnieniu bufora leci od poczatku w kolko
	DMA_Init(DMA1_Channel1, &dma); //dma1 kanal1 (maksymalnie jest 12 kanalow)
	DMA_Cmd(DMA1_Channel1, ENABLE); //wlaczamy dma1 kanal1
	//konfiguracja ADC
	ADC_StructInit(&adc);
	adc.ADC_ScanConvMode = ENABLE; //skanowanie wielokanalowe
	adc.ADC_ContinuousConvMode = ENABLE; //ciagle dzialanie
	adc.ADC_NbrOfChannel = ADC_CHANNELS; //ilosc kanalow
	adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_Init(ADC1, &adc);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); //ustawianie kanalow ADC //czujnik1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5); //czujnik2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_239Cycles5); //pozycja serwa
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 4, ADC_SampleTime_239Cycles5); //prad w serwie
	ADC_DMACmd(ADC1, ENABLE); //uruchomienie kanalu DMA przypisanego do ADC
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1); //start kalibracji ADC
	while (ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while (ADC_GetCalibrationStatus(ADC1)); //koniec kalibracji ADC
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //wlaczenie konwersji ADC
	//ADC_TempSensorVrefintCmd(ENABLE); // wlaczamy napiecie referencyjne (jest ono na kanale 17)

	//konfiguracja timera do PWM
	TIM_TimeBaseStructInit(&tim);
	tim.TIM_CounterMode = TIM_CounterMode_Up; //zliczanie w gore (od 0 do period)
	tim.TIM_Prescaler = 64 - 1; //co ile tickow jedno podliczenie
	tim.TIM_Period = 20000 - 1; //naliczanie do 20ms
	TIM_TimeBaseInit(TIM4, &tim);
	//kanal
	TIM_OCStructInit(&channel);
	channel.TIM_OCMode = TIM_OCMode_PWM1; //jako PWM
	channel.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM4, &channel); //kanal 3
	TIM_Cmd(TIM4, ENABLE); //wlaczenie timera4

	//konfiguracja timera do wysylania danych do PC
	tim.TIM_Prescaler = 64000 - 1;
	tim.TIM_Period = (uint16_t)(1000*0.1 - 1); //wysylanie danych do PC domyslnie co 0.1 [s]
	TIM_TimeBaseInit(TIM3, &tim);
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //uruchomienie przerwania
	TIM_Cmd(TIM3, ENABLE); //wlaczenie timera3
	//kontroler przerwan do wysylania danych do PC
	nvic.NVIC_IRQChannel = TIM3_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	//konfiguracja timera do liczenia nastaw itp. itd.
	tim.TIM_Prescaler = 64000 - 1;
	tim.TIM_Period = (uint16_t)(1000*0.1 - 1); //oblczanie nastaw co okres probkowania T - domyslnie co 0.1 [s]
	TIM_TimeBaseInit(TIM2, &tim);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //uruchomienie przerwania
	TIM_Cmd(TIM2, ENABLE); //wlaczenie timera2
	//kontroler przerwan do liczenia nastaw itp. itd.
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 0;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);

	while (1) {

	}
}
