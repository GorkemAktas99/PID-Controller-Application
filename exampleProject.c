/*******************************************************************************
|==============================================================================|
|	@author Gorkem Aktas                                                         |
|	@date 25 January 2021            	                                           |
|	@documentedDate 25 January 2021	                                             |
|	@brief This experiment aims to realize a P and PI type controller application|
|==============================================================================|
*******************************************************************************/
#include "stm32f10x.h"                  // Device header
#include "stm32f10x_adc.h"              // Keil::Device:StdPeriph Drivers:ADC
#include "stm32f10x_usart.h"
#include <stdio.h>
char message[20]; // COM Port Message variable
uint16_t adcValue=0; // For reading analog signal
float outputSignal = 0;
float pwmController = 1;
double outputController = 0 , outputController1 = 0;
int buttonState = 0;
//Function Signs for using in EXTI Interrupt function
void uartTransmit(char *string);
double PID(double r, double y);
double PID1(double r, double y);
void delay(uint32_t time );
volatile double outputToPC;
volatile double inputToSystem = 1;
volatile double stepSize = 0.01;



void gpioConfig(){ //GPIO Pins Configuration 
	GPIO_InitTypeDef GPIOInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE); // PORTA communication line is activated
	
	GPIOInitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
  GPIOInitStructure.GPIO_Pin=GPIO_Pin_1;
  GPIOInitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&GPIOInitStructure);
	
	GPIOInitStructure.GPIO_Mode=GPIO_Mode_IPD;
  GPIOInitStructure.GPIO_Pin=GPIO_Pin_0;
  GPIOInitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&GPIOInitStructure);
	
	GPIOInitStructure.GPIO_Mode=GPIO_Mode_AIN;
	GPIOInitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_Init(GPIOA,&GPIOInitStructure); //Initialize PA6

	//TX->PA9 AND RX->PA10
  //TX
  GPIOInitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
  GPIOInitStructure.GPIO_Pin=GPIO_Pin_9;
  GPIOInitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA,&GPIOInitStructure);
  //RX
  GPIOInitStructure.GPIO_Mode=GPIO_Mode_IN_FLOATING;
  GPIOInitStructure.GPIO_Pin=GPIO_Pin_10;
  GPIO_Init(GPIOA,&GPIOInitStructure);
}
void extiConfig(){
	EXTI_InitTypeDef EXTIInitStructure; //Object called for EXTI
	NVIC_InitTypeDef NVICInitStructure; //Object called for NVIC
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //Connection line for EXTI has been activated. 
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource0); //Port and pin to be given priority are set
	
	//Configure external interrupt 
	EXTI_DeInit();
	EXTIInitStructure.EXTI_Line=EXTI_Line0; // Line0 is used
	EXTIInitStructure.EXTI_LineCmd=ENABLE; // Line is enable
	EXTIInitStructure.EXTI_Mode=EXTI_Mode_Interrupt; // Line mode for interrupt
	EXTIInitStructure.EXTI_Trigger=EXTI_Trigger_Rising; // Rising Trigger for press button
	//Set rising to enable interrupt as soon as the button is pressed
	
	EXTI_Init(&EXTIInitStructure);
	
	//NVIC part
	NVICInitStructure.NVIC_IRQChannel=EXTI0_IRQn; 
	NVICInitStructure.NVIC_IRQChannelCmd=ENABLE;
	//Priority set for interrupt
	NVICInitStructure.NVIC_IRQChannelPreemptionPriority=0x00;
	NVICInitStructure.NVIC_IRQChannelSubPriority=0x00;
	
	NVIC_Init(&NVICInitStructure);


}
void EXTI0_IRQHandler(){
	
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET){ //It will work if the signal on the EXTI line is different from RESET.
		
		if(buttonState == 0){
			buttonState = 1;
			delay(1000000);
			inputToSystem = 1;

		}
		else if(buttonState == 1){
			buttonState = 0;
			delay(1000000);
			inputToSystem = 0;
		}
   EXTI_ClearITPendingBit(EXTI_Line0);
	
	}
}
void adcConfig(){ //Analog-Digital Converter Configuration
	ADC_InitTypeDef ADCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE); //APB2 Line is activated for ADC
	
	ADCInitStructure.ADC_ContinuousConvMode=ENABLE; // Continuous cycle is provided
	ADCInitStructure.ADC_DataAlign=ADC_DataAlign_Right; // The data are given based on the right.
	ADCInitStructure.ADC_ExternalTrigConv=ADC_ExternalTrigConv_None; //Disable because no trigger is used
	ADCInitStructure.ADC_Mode=ADC_Mode_Independent; 
	ADCInitStructure.ADC_NbrOfChannel=1; //Single channel
	ADCInitStructure.ADC_ScanConvMode=DISABLE; // Since we do not use Multi-Channel, it has been disabled.
	
	ADC_Init(ADC1,&ADCInitStructure); //Initialize ADC
	ADC_Cmd(ADC1,ENABLE); // ADC Start Command

}
uint16_t readADC(){ // Analog data is read
	
	ADC_RegularChannelConfig(ADC1,ADC_Channel_6,1,ADC_SampleTime_55Cycles5); //Channel tuning for ADC
	ADC_SoftwareStartConvCmd(ADC1,ENABLE); //ADC started
	
	while(ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC)==RESET); //If the ADC does not read any value, it will run blank.
	
	return ADC_GetConversionValue(ADC1); // If the ADC reads a value, the value will be returned.

}
//The data we want with the data coming from the ADC is proportioned by this function.
float map(float adcValue, float max, float min, float conMax, float conMin){
	
	return adcValue*((conMax-conMin)/(max-min));

}
void uartConfig(){
   USART_InitTypeDef UARTInitStructure;
   
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
   
   UARTInitStructure.USART_BaudRate=9600;
   UARTInitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
   UARTInitStructure.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;
   UARTInitStructure.USART_Parity=USART_Parity_No;
   UARTInitStructure.USART_StopBits=USART_StopBits_1;
   UARTInitStructure.USART_WordLength=USART_WordLength_8b;
   
   USART_Init(USART1,&UARTInitStructure);
   USART_Cmd(USART1,ENABLE);
}
void uartTransmit(char *string){
	
   while(*string){
      while(!(USART1->SR & 0x00000040));
      USART_SendData(USART1,*string);
      *string++;

   }
}
void delay(uint32_t time ){

   while(time--);


}
void timerConfig(){
   
   TIM_TimeBaseInitTypeDef TIMERInitStructure;
   
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
   
   TIMERInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
   TIMERInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
   TIMERInitStructure.TIM_Prescaler=10;
   TIMERInitStructure.TIM_Period=2399;
   TIMERInitStructure.TIM_RepetitionCounter=0;
   
   TIM_TimeBaseInit(TIM2,&TIMERInitStructure);
   TIM_Cmd(TIM2,ENABLE);

}
void pwmConfig(uint32_t timPulse){
   
   TIM_OCInitTypeDef TIMER_OCInitStructure;
   TIMER_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
   TIMER_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
   TIMER_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
   TIMER_OCInitStructure.TIM_Pulse=timPulse;

   TIM_OC2Init(TIM2,&TIMER_OCInitStructure);
   TIM_OC2PreloadConfig(TIM2,TIM_OCPreload_Enable);

}
void samplingTimerConfig(){
	
	TIM_TimeBaseInitTypeDef TIMERInitStructure;
	NVIC_InitTypeDef        NVICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIMERInitStructure.TIM_ClockDivision=1;
	TIMERInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIMERInitStructure.TIM_Period=144;
	TIMERInitStructure.TIM_Prescaler=5000;
	TIMERInitStructure.TIM_RepetitionCounter=0;
	
	TIM_TimeBaseInit(TIM3,&TIMERInitStructure);
	TIM_Cmd(TIM3,ENABLE);
	
	NVICInitStructure.NVIC_IRQChannel=TIM3_IRQn;
	NVICInitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVICInitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVICInitStructure.NVIC_IRQChannelSubPriority=0;
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	NVIC_Init(&NVICInitStructure);
}
void samplingTimer4Config(){
	
	TIM_TimeBaseInitTypeDef TIMERInitStructure;
	NVIC_InitTypeDef        NVICInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	TIMERInitStructure.TIM_ClockDivision=1;
	TIMERInitStructure.TIM_CounterMode=TIM_CounterMode_Up;
	TIMERInitStructure.TIM_Period=144;
	TIMERInitStructure.TIM_Prescaler=5000;
	TIMERInitStructure.TIM_RepetitionCounter=0;
	
	TIM_TimeBaseInit(TIM4,&TIMERInitStructure);
	TIM_Cmd(TIM4,ENABLE);
	
	NVICInitStructure.NVIC_IRQChannel=TIM4_IRQn;
	NVICInitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVICInitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVICInitStructure.NVIC_IRQChannelSubPriority=0;
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	NVIC_Init(&NVICInitStructure);
}
void TIM3_IRQHandler(){
//		outputController = 1;
		outputController = PID(inputToSystem,outputSignal);
		pwmConfig(730*outputController*buttonState);
		adcValue=readADC(); // Reading potentiometer
		outputSignal = map(adcValue,4030,0,3.3,0)-0.02; // Mapping analog data
		if(outputSignal > 0){
			sprintf(message,"%.2f\n",outputSignal);
			uartTransmit(message);
		}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
// System parameters
double K = 1, zeta = 1.4158, wn = 0.6742;
// Differential Equation y1'(t) = y2
double eq1(double y1, double y2, double u)
{
 return y2;
}
double eq2(double y, double y1, double u)
{
 return (K*wn*wn*u)-(2*zeta*wn*y)-(wn*wn*y1);
}
double ExplicitEulerEq1(double (*f)(double,double,double), double y2, double ts, double uk){
 static double yk = 0;
 double yk1 = yk + ts * f(yk, y2, uk);
 yk = yk1;
 return yk1;
}
double ExplicitEulerEq2(double (*f)(double,double,double), double y1, double ts, double uk){
 static double yk = 0; // Previous value of the output
 double yk1 = yk + ts * f(yk, y1, uk);
 yk = yk1;
 return yk1;
}
//volatile double outputToPC;
//volatile double inputToSystem = 1;
//volatile double stepSize = 0.1;

void TIM4_IRQHandler(void)
{
 static double y2=0, y1=0; // Initial values
 if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) // Periodic IRQ
 {
	outputController1 = PID1(inputToSystem,outputToPC);
	y1 = ExplicitEulerEq1(eq1, y2, stepSize, outputController1);
	y2 = ExplicitEulerEq2(eq2, y1, stepSize, outputController1);
	outputToPC = y1;
	sprintf(message,"%.2f\n",outputToPC);
	uartTransmit(message);
//	// Clear interrupt flag
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
 }
}
volatile double Kp = 2.8, Ki = 0.8, Kd = 0;
volatile double Kp1 = 2.8, Ki1 = 0.8, Kd1 = 0;

double Derivative(double ek, double ek_1)
{
 return (ek - ek_1) / stepSize;
}
double Integral(double ek, double ek_1)
{
 return (ek + ek_1) * (stepSize / 2);
}
double PID(double r, double y)
{
 static double ek_1 = 0, integralSum = 0;
 double outputPID;
 double ek = r - y;
	if(buttonState == 0){
		integralSum = 0;
	}
 integralSum += Integral(ek, ek_1);
 outputPID = Kp * ek + Ki * integralSum + Kd * Derivative(ek, ek_1);
 ek_1 = ek;
 return outputPID;
}
double PID1(double r, double y)
{
 static double ek_1 = 0, integralSum = 0;
 double outputPID;
 double ek = r - y;
 integralSum += Integral(ek, ek_1);
 outputPID = Kp1 * ek + Ki1 * integralSum + Kd1 * Derivative(ek, ek_1);
 ek_1 = ek;
 return outputPID;
}
int main(){
	gpioConfig(); //GPIO pins are activated
	adcConfig(); //ADC pin is activated
	uartConfig();
	timerConfig();
	samplingTimerConfig();
	samplingTimer4Config();
	extiConfig();
	while(1){

		
	}
}
