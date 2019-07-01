#include "main.h"

#define POWER_KEY (1-HAL_GPIO_ReadPin(Power_Key_GPIO_Port,Power_Key_Pin))
//#define RASPI_STATUS	HAL_GPIO_ReadPin(RPI_PW_STAT_GPIO_Port,RPI_PW_STAT_Pin)
#define RASPI_STATUS	HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_7)

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
volatile float POWER_LED_PWM=0;
volatile float STANDBY_LED_PWM=0;
int pw_state=1;
int ledFlashMode;
volatile float power_led_pwm_val=0;
volatile float standby_led_val=0;

void led_interrupt_rutine(void)
{
	static int cnt=0;
	cnt++;
	if(cnt>9)
	{
		cnt=0;
		power_led_pwm_val=0.9*power_led_pwm_val+0.1*POWER_LED_PWM;
		standby_led_val=0.9*standby_led_val+0.1*STANDBY_LED_PWM;
		TIM2->CCR4=(uint16_t)(power_led_pwm_val);
		TIM3->CCR1=(uint16_t)(standby_led_val);
	}
}
void powerManager_init(void)
{
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
}
enum {POWER_ON , STANDBY , POWER_OFF , SHUTING_DOWN};
		void setBackLight(int state){
			if(state==1)
				HAL_GPIO_WritePin(BL_EN_GPIO_Port,BL_EN_Pin,GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(BL_EN_GPIO_Port,BL_EN_Pin,GPIO_PIN_RESET);
		}
		void setRaspPower(int state){
			if(state==0)
				HAL_GPIO_WritePin(RPi_PW_GPIO_Port,RPi_PW_Pin,GPIO_PIN_SET);
			else
				HAL_GPIO_WritePin(RPi_PW_GPIO_Port,RPi_PW_Pin,GPIO_PIN_RESET);
		}

		void ledStateMachine(void){
			volatile static int state=0,led_timer=0,state1=0;
			if(ledFlashMode==POWER_ON){
				POWER_LED_PWM=7200;
				STANDBY_LED_PWM=0;
			
			}else if(ledFlashMode == STANDBY){
				STANDBY_LED_PWM=0;
				switch(state){
					case 0:
						led_timer=200;
					  POWER_LED_PWM=7200;
						state=1;
						break;
					case 1:
						if(led_timer<=0){
							led_timer=1000;
							POWER_LED_PWM=0;
							state=2;
						}else{
							led_timer--;
							state=1;
						}
						break;
					case 2:
						if(led_timer<=0)
							state=0;
						else{
							led_timer--;
							state=2;
						}
							break;
					default:
						state=0;
					break;
				}
			
			}else if(ledFlashMode==POWER_OFF){
				POWER_LED_PWM=0;
				STANDBY_LED_PWM=7200;
			}else{//Shuting Down
				switch(state1){
					case 0:
						led_timer=300;
					  POWER_LED_PWM=7200;
						state1=1;
						break;
					case 1:
						if(led_timer<=0){
							led_timer=300;
							POWER_LED_PWM=-7200;
							state1=2;
						}else{
							led_timer--;
							state1=1;
						}
						break;
					case 2:
						if(led_timer<=0)
							state1=0;
						else{
							led_timer--;
							state1=2;
						}
							break;
					default:
						state1=0;
					break;
				}//END of SWITCH
			}

		}
		void poweSwitchStateMachine(void){
		volatile static int pw_state=1,timer=0;
				switch(pw_state){
		case 1:
			if(POWER_KEY==1){
				pw_state=12;
			}else{
				pw_state=1;
				ledFlashMode=POWER_OFF;
			}
			break;
		case 12:
			if(POWER_KEY==1)
				pw_state=12;
			else{
				pw_state=2;
				ledFlashMode=POWER_ON;
				setRaspPower(1);
				//setBackLight(1);
			}
			break;
		case 2:
			if(POWER_KEY==1){
				timer=1000;
				pw_state=3;
			}else
				pw_state=101;
			break;
			
		case 101:
				if(RASPI_STATUS==1)
					setBackLight(1);
				  pw_state=2;
		break;
		case 3:
				if((timer>0)&&(POWER_KEY==1)){
					timer--;
					pw_state=3;
				}else{
					pw_state=5;
				}
				break;
		case 5:
			if(timer<=0){
				HAL_GPIO_WritePin(RPI_SPW_GPIO_Port,RPI_SPW_Pin,GPIO_PIN_RESET);
				HAL_Delay(500);
				ledFlashMode=SHUTING_DOWN;
				pw_state=7;
			}else{
				pw_state=6;
			}
			break;
		case 7:
			HAL_GPIO_WritePin(RPI_SPW_GPIO_Port,RPI_SPW_Pin,GPIO_PIN_SET);
			timer=7500;
			pw_state=8;
			break;
		case 6:
			if(timer<950){
				pw_state=14;
				setBackLight(0);
				ledFlashMode=STANDBY;
			}
			else
				pw_state=2;	
			break;
		case 14:
			if(POWER_KEY==1)
				pw_state=14;
			else
				pw_state=11;
			break;
		case 11:
			if(POWER_KEY==1){
				ledFlashMode=POWER_ON;
				setBackLight(1);
				pw_state=15;
			}else
			 pw_state=11;
			break;
		case 15:
			if(POWER_KEY==1)
				pw_state=15;
			else
				pw_state=2;
			break;
		case 8:
			if((RASPI_STATUS==1)&&(timer>0)){
				timer--;
				pw_state=8;
			}else{
				setBackLight(0);
				timer=8000;
				pw_state=9;
			}
			break;
		case 9:
			if(timer<=0){
				setRaspPower(0);
				setBackLight(0);
				ledFlashMode=POWER_OFF;
				pw_state=13;
			}else{
				pw_state=9;
				timer--;
			}
			break;
		case 13:
			if(POWER_KEY==1)
				pw_state=13;
			else
				pw_state=1;
			break;
			
		default:
			pw_state=1;
			break;	
	}
	  
		}
/* USER CODE END 0 */
		
