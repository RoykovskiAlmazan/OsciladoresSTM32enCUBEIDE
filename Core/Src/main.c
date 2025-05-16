/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *


          _           _    _        _          _              _      _          _       _            _               _
        /\ \        /\ \ /\ \     /\_\       /\_\           /\ \   /\ \    _ / /\     / /\         /\_\            /\ \
       /  \ \      /  \ \\ \ \   / / /      / / /  _       /  \ \  \ \ \  /_/ / /    / /  \       / / /  _         \ \ \
      / /\ \ \    / /\ \ \\ \ \_/ / /      / / /  /\_\    / /\ \ \  \ \ \ \___\/    / / /\ \__   / / /  /\_\       /\ \_\
     / / /\ \_\  / / /\ \ \\ \___/ /      / / /__/ / /   / / /\ \ \ / / /  \ \ \   / / /\ \___\ / / /__/ / /      / /\/_/
    / / /_/ / / / / /  \ \_\\ \ \_/      / /\_____/ /   / / /  \ \_\\ \ \   \_\ \  \ \ \ \/___// /\_____/ /      / / /
   / / /__\/ / / / /   / / / \ \ \      / /\_______/   / / /   / / / \ \ \  / / /   \ \ \     / /\_______/      / / /
  / / /_____/ / / /   / / /   \ \ \    / / /\ \ \     / / /   / / /   \ \ \/ / /_    \ \ \   / / /\ \ \        / / /
 / / /\ \ \  / / /___/ / /     \ \ \  / / /  \ \ \   / / /___/ / /     \ \ \/ //_/\__/ / /  / / /  \ \ \   ___/ / /__
/ / /  \ \ \/ / /____\/ /       \ \_\/ / /    \ \ \ / / /____\/ /       \ \  / \ \/___/ /  / / /    \ \ \ /\__\/_/___\
\/_/    \_\/\/_________/         \/_/\/_/      \_\_\\/_________/         \_\/   \_____\/   \/_/      \_\_\\/_________/

									¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									¶¶¶¶¶¶¶¶¶¶1_________________1¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									¶¶¶¶¶¶¶¶1___111_1111111_1____¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									¶¶¶¶¶¶¶1_1881188111111111____¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									¶¶¶¶¶¶881188__¶___8111_______¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									¶¶8881____1__________________¶¶¶¶¶¶¶¶¶888¶¶¶¶
									¶11188¶¶¶¶¶¶¶¶¶¶881__________818¶¶¶¶¶¶¶¶18¶¶¶
									¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶81___1¶_1¶¶¶¶¶¶¶¶¶11¶¶
									¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶818¶¶¶¶¶¶¶¶¶¶¶¶18¶
									¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶8888¶¶¶¶¶¶¶¶81¶
									¶¶¶¶¶¶¶¶¶¶¶¶¶88888¶8¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶81¶
									¶¶¶¶¶¶¶¶881111111111_11118888¶¶¶¶¶¶¶¶¶¶¶¶¶81¶
									¶¶¶¶¶881881181111118¶¶¶88111111_____8¶¶¶¶¶¶1¶
									¶¶¶¶¶8¶¶¶81111118¶¶¶¶¶81111881111_____1¶¶¶¶1¶
									¶¶¶¶¶¶¶¶¶¶¶811118¶81111__118111111111___8¶8_¶
									¶¶¶¶¶¶¶¶¶¶¶¶¶1__1888¶¶¶¶¶¶¶8______11181___18¶
									¶¶¶¶¶¶81118¶¶8_____18188881_______11181__888¶
									¶¶¶¶¶¶¶¶¶¶8881____________________1118118¶__¶
									¶¶¶¶¶¶¶¶881_81___________________111181_188_¶
									¶¶¶¶¶¶¶¶81_1¶8__________________1111881_____¶
									¶¶¶¶¶8111_118______11_________1111888811____¶
									¶¶¶¶¶81111181__18¶8111___1_111111811111_____¶
									¶¶¶¶¶¶8111_8¶¶¶¶¶¶¶8______1111111111111___18¶
									¶¶¶¶¶¶¶81111¶¶¶¶1____________111111111¶¶¶¶¶¶¶
									¶¶¶¶¶¶¶¶8111¶¶1_______________1111111888¶¶¶8¶
									¶¶¶¶¶¶¶¶¶81118¶88¶¶¶8881_____1111111188_18¶_¶
									¶¶¶¶¶¶¶¶¶¶818¶¶8111__1___111111111118¶811188¶
									¶¶¶¶¶¶¶¶¶¶¶8188888¶¶881111181111118¶¶8111188¶
									¶¶¶¶¶¶¶¶¶¶¶¶8¶¶¶¶¶¶811111111111188¶¶81111181¶
									¶¶¶¶¶¶8888¶¶¶¶¶8____________188¶¶¶¶8111111_1¶
									¶¶¶¶81111¶¶¶¶¶811_1111___118¶¶¶¶¶¶81111111_¶¶
									¶¶¶811_1¶¶¶¶¶¶¶88888¶¶¶¶8¶¶¶¶¶¶¶¶81111111_8¶¶
									¶81___1¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶88811118__¶¶¶
									¶___8¶¶¶88¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶888881888_¶8_¶
									¶81_¶¶8888¶¶¶8¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶888888888__¶8__¶
									¶¶¶¶1__1¶¶¶888¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶888888881___1¶¶¶
									¶¶88¶¶8_81_1¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶8¶88881_11¶¶¶88¶
									¶¶1_18¶¶8__¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶88__8¶¶¶¶1_1¶
									¶81_1118¶¶¶__1¶¶¶¶¶¶¶8¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶¶
									888888888888888888888888888888888888888888888
									8___________________________________________8
									8__¶¶¶¶¶_¶¶___¶¶_¶¶¶¶_¶¶__¶¶_¶¶¶¶¶_¶¶___¶¶__8
									8__¶¶____¶¶¶_¶¶¶__¶¶__¶¶¶_¶¶_¶¶____¶¶¶_¶¶¶__8
									8__¶¶¶¶__¶¶_¶_¶¶__¶¶__¶¶_¶¶¶_¶¶¶¶__¶¶_¶_¶¶__8
									8__¶¶____¶¶___¶¶__¶¶__¶¶__¶¶_¶¶____¶¶___¶¶__8
									8__¶¶¶¶¶_¶¶___¶¶_¶¶¶¶_¶¶__¶¶_¶¶¶¶¶_¶¶___¶¶__8
									8___________________________________________8
									888888888888888888888888888888888888888888888



  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_ID0 0
#define SERVO_ID1 1
#define SERVO_ID2 2
#define SERVO_ID3 3
#define SERVO_ID4 4
#define SERVO_ID5 5
#define SERVO_ID6 6
#define SERVO_ID7 7
#define SERVO_ID8 8
#define SERVO_ID9 9
#define SERVO_ID10 10
#define SERVO_ID11 11
#define GOAL_POSITION_L 0x1E //posible solucion al problema del espejo >:v

		//Definimos el tamano de los arreglos
#define N1 3
#define N2 2
#define N3 2
#define N4 4

#ifndef PI
#define PI 3.14159265358979323846f
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float alpha;
//const float omega = 2* PI;
float theta = 0.0f;
const float beta1 = -20.1173;
const float beta2 = -11.7457;
const float beta3 = -66.9718;
const float beta4 = -3.3952;
const float z0_1 = 0.6;
const float z0_2 = 0.75;
const float z0_3 = 1;
const float z0_4 = 0.43;


// OSCILADOR 1 Codo
const float a1[N1] = {-98.7690f, 92.8651f, 22.4126f};
const float b1[N1] = {0.0666f, 0.1153f, 0.4078f};
const float deltaTheta1[N1] = {0.6471f, 1.4800f, 4.9683f};


// OSCILADOR 2
/*
const float a2[N2] = {-15.8419f, 20.4244f};
const float b2[N2] = {0.1829f, 0.7849f};
const float deltaTheta2[N2] = {4.2979f, 5.3540f};*/

// OSCILADOR 3 Rodilla
const float a3[N3] = {-84.8447f, 35.2495f};
const float b3[N3] = {0.4260f, 0.8688f};
const float deltaTheta3[N3] = {4.3614f, 5.6522f};

// OSCILADOR 4 Tarso
const float a4[N4] = {6.8057f, -5.7171f, 2.4815f, -10.9075f};
const float b4[N4] = {0.0287f, 4.4220f, 3.5778f, 0.4546f};
const float deltaTheta4[N4] = {0.0014f, 0.5201f, 2.0669f, 3.8113f};
//Inicializacion de los patametros para el oscilador


float x = 1.0f, y = 0.0f;
float z3 = z0_3, z4 = z0_4, z1 = z0_3, z2 = z0_4, z5 = z0_3, z6 = z0_4,z7 = z0_3, z8 = z0_4;
float dt = 0.001f;  // Paso de integración
float omega = 2.0f * PI;
float z3_grados;
float z4_grados;
float z1_grados;
float z2_grados;
float z5_grados;
float z6_grados;
float z7_grados;
float z8_grados;
float z3_posicion;
float z4_posicion;
float z1_posicion;
float z2_posicion;
float z5_posicion;
float z6_posicion;
float z7_posicion;
float z8_posicion;


float sumatoria3 = 0.0f;
float atractores3 = 0.0f;
float sumatoria4 = 0.0f;
float atractores4 = 0.0f;
float atractores1 = 0.0f;
float atractores2 = 0.0f;
float sumatoria2 = 0.0f;
float sumatoria1 = 0.0f;
float atractores5 = 0.0f;
float atractores6 = 0.0f;
float sumatoria5 = 0.0f;
float sumatoria6 = 0.0f;
float atractores7 = 0.0f;
float atractores8 = 0.0f;
float sumatoria7 = 0.0f;
float sumatoria8 = 0.0f;
float x_new, y_new, z3_new, z4_new,z1_new, z2_new, z5_new, z6_new, z7_new, z8_new;

void calcular_z3_z4() {
	theta = atan2f(y, x);
	        if (theta < 0.0f) { //Calculo de tyheta
	            theta += 2.0f * PI;
	}
	        	/*
	        	 *
 __       ___          __   ___                ___  ___  __            __  __          ___  __   __
|__)  /\   |   /\     |  \ |__  |     /\  |\ |  |  |__  |__)  /\     |  / /  \ |  | | |__  |__) |  \  /\
|    /~~\  |  /~~\    |__/ |___ |___ /~~\ | \|  |  |___ |  \ /~~\    | /_ \__X \__/ | |___ |  \ |__/ /~~\
	        	 *
	        	 * */
	        	atractores3 = 0.0f;
	            for (int i = 0; i < N1; i++) {
	            	atractores3 += a1[i] * expf(-b1[i] * fabsf(theta - deltaTheta1[i]));
	            }

	            atractores4 = 0.0f;
	            for (int i = 0; i < N4; i++) {
	            	atractores4 += a4[i] * expf(-b4[i] * fabsf(theta - deltaTheta4[i]));
	            }
	            /*
	             *
 __       ___         ___  __        __   ___  __            __  __          ___  __   __
|__)  /\   |   /\      |  |__)  /\  /__` |__  |__)  /\     |  / /  \ |  | | |__  |__) |  \  /\
|    /~~\  |  /~~\     |  |  \ /~~\ .__/ |___ |  \ /~~\    | /_ \__X \__/ | |___ |  \ |__/ /~~\
	             *
	             * */

	            atractores1 = 0.0f;
				for (int i = 0; i < N1; i++) {
					atractores1 += a1[i] * expf(-b1[i] * fabsf(fmodf(theta + PI / 2, 2 * PI) - deltaTheta1[i]));
				}

				// C�?LCULO DE ATRACTORES PARA OSCILADOR 4
				atractores2 = 0.0f;
				for (int i = 0; i < N4; i++) {
					atractores2 += a4[i] * expf(-b4[i] * fabsf(fmodf(theta + PI / 2, 2 * PI) - deltaTheta4[i]));
				}
/*
 *
 __       ___          __   ___                ___  ___  __           __   ___  __   ___  __
|__)  /\   |   /\     |  \ |__  |     /\  |\ |  |  |__  |__)  /\     |  \ |__  |__) |__  /  ` |__|  /\
|    /~~\  |  /~~\    |__/ |___ |___ /~~\ | \|  |  |___ |  \ /~~\    |__/ |___ |  \ |___ \__, |  | /~~\
 *
 * */



				atractores7 = 0.0f;
				for (int i = 0; i < N1; i++) {
					atractores7 += a1[i] * expf(-b1[i] * fabsf(fmodf(theta + PI, 2 * PI) - deltaTheta1[i]));
				}

				// C�?LCULO DE ATRACTORES PARA OSCILADOR 4
				atractores8 = 0.0f;
				for (int i = 0; i < N4; i++) {
					atractores8 += a4[i] * expf(-b4[i] * fabsf(fmodf(theta + PI, 2 * PI) - deltaTheta4[i]));
				}



				/*
				 *
 __       ___         ___  __        __   ___  __           __   ___  __   ___  __
|__)  /\   |   /\      |  |__)  /\  /__` |__  |__)  /\     |  \ |__  |__) |__  /  ` |__|  /\
|    /~~\  |  /~~\     |  |  \ /~~\ .__/ |___ |  \ /~~\    |__/ |___ |  \ |___ \__, |  | /~~\
				 *
				 * */
				float theta_desfasado = fmodf(theta + 3 * PI / 2, 2 * PI);


				 atractores5 = 0.0f;
				for (int i = 0; i < N1; i++) {
					atractores5 += a1[i] * expf(-b1[i] * fabsf(theta_desfasado - deltaTheta1[i]));
				}

				// C�?LCULO DE ATRACTORES PARA OSCILADOR 4
				atractores6 = 0.0f;
				for (int i = 0; i < N4; i++) {
					atractores6 += a4[i] * expf(-b4[i] * fabsf(theta_desfasado - deltaTheta4[i]));

				}




	            // DIN�?MICA DE OSCILADORES (MÉTODO DE EULER)

	            // Cálculo de alpha
	            alpha = 1.0f - sqrtf(x * x + y * y);

	            // Oscilador base (x, y)
	            x_new = x + dt * (alpha * x - omega * y);
	            y_new = y + dt * (alpha * y + omega * x);

	            // Oscilador 3
	            float primerTermino3 = -beta3 * (z3 - z0_1);
	            float dzdt3 = -primerTermino3 + atractores3;
	            z3_new = z3 + dzdt3 * dt;

	            // Oscilador 4
	            float primerTermino4 = -beta4 * (z4 - z0_4);
	            float dzdt4 = -primerTermino4 + atractores4;
	            z4_new = z4 + dzdt4 * dt;


	            // Oscilador 3
				float primerTermino1 = -beta3 * (z1 - z0_1);
				float dzdt1 = -primerTermino1 + atractores1;
				z1_new = z1 + dzdt1 * dt;

				// Oscilador 4
				float primerTermino2 = -beta4 * (z2 - z0_4);
				float dzdt2 = -primerTermino2 + atractores2;
				z2_new = z2 + dzdt2 * dt;

				float primerTermino5 = -beta3 * (z5 - z0_1);
				float dzdt5 = -primerTermino5 + atractores5;
				z5_new = z5 + dzdt5 * dt;

				// Oscilador 4
				float primerTermino6 = -beta4 * (z6 - z0_4);
				float dzdt6 = -primerTermino6 + atractores6;
				z6_new = z6 + dzdt6 * dt;


				// Oscilador 3
				float primerTermino7 = -beta3 * (z7 - z0_1);
				float dzdt7 = -primerTermino7 + atractores7;
				z7_new = z7 + dzdt7 * dt;

				// Oscilador 4
				float primerTermino8 = -beta4 * (z8 - z0_4);
				float dzdt8 = -primerTermino8 + atractores8;
				z8_new = z8 + dzdt8 * dt;

	                // ACTUALIZAR ESTADO
	            x = x_new;
	            y = y_new;

	            z3 = z3_new;
	            z4 = z4_new;

	            z1 = z1_new;
	            z2 = z2_new;

	            z5 = z5_new;
				z6 = z6_new;

				z7 = z7_new;
				z8 = z8_new;

}


void enviar_dato(uint8_t* dato, uint8_t size ){

	HAL_UART_Transmit(&huart2, dato, size, HAL_MAX_DELAY);
}

void syncwrite_mover_servos(uint8_t *ids, uint16_t *posiciones, uint8_t cantidad){
	const uint8_t INSTRUCCION = 0x83; //FUNCION SYNWRITE EN HEZADECIMAL UWU
	const uint8_t ID_BROADCAST = 0xFE;
	const uint8_t START_ADDR = GOAL_POSITION_L;
	const uint8_t BYTES_POR_SERVO = 3; //es el id + 2 bytes de posicion
	// algo como 4 headers + N *3(id + 2 bytes) + 1 byte de cheksum

	uint8_t longitud = 4+ cantidad*BYTES_POR_SERVO;
	uint8_t paquete[6 + cantidad * BYTES_POR_SERVO];

	int i = 0;
	    paquete[i++] = 0xFF;
	    paquete[i++] = 0xFF;
	    paquete[i++] = ID_BROADCAST;
	    paquete[i++] = longitud;
	    paquete[i++] = INSTRUCCION;
	    paquete[i++] = START_ADDR;
	    paquete[i++] = 2; // 2 bytes por dato (posición)

	    for (uint8_t j = 0; j < cantidad; j++) {
	            uint16_t pos = posiciones[j];
	            paquete[i++] = ids[j];         // ID del servo
	            paquete[i++] = pos & 0xFF;     // LSB
	            paquete[i++] = (pos >> 8) & 0xFF; // MSB
	        }
	    //cheksum

	    uint8_t checksum = 0;
	       for (int j = 2; j < i; j++) {
	           checksum += paquete[j];
	       }
	       checksum = ~checksum;
	       paquete[i++] = checksum;

	       // Enviar por UART
	       enviar_dato(paquete, i);
}

uint16_t grados_a_posicion(float grados) {
    if (grados < 0.0f) grados = 0.0f;
    if (grados > 300.0f) grados = 300.0f;
    return (uint16_t)(grados * (1023.0f / 300.0f));
}





/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  uint8_t id[] = {0, 1, 2,3,4,5,6,7,8,9,10,11};
  uint16_t posicion[] = {512, 512, 410,512, 512, 410,512, 512, 410,512, 512, 410};


  syncwrite_mover_servos(id, posicion, 12);

  HAL_Delay(3000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
    calcular_z3_z4();

        z3_grados = (z3 * 40 + 135);		//24 +48 funciona decente
        z4_grados = z4 * 38  + 98;				//DI 38 + 98 funciona descente

        z7_grados = (z7 * 40 + 135);
        z8_grados = z8 * 38 + 98;				//DD

        z1_grados = (z1 * 40 + 135);
        z2_grados = z2 * 38  + 98;				//TI

        z5_grados = (z5 * 40 + 135);			//TD
        z6_grados = z6 * 38  + 98;



        z3_posicion = grados_a_posicion(z3_grados);
        z4_posicion = grados_a_posicion(z4_grados);
        z1_posicion = grados_a_posicion(z1_grados);
        z2_posicion = grados_a_posicion(z2_grados);

        z5_posicion = grados_a_posicion(z5_grados);
        z6_posicion = grados_a_posicion(z6_grados);
        z7_posicion = grados_a_posicion(z7_grados);
        z8_posicion = grados_a_posicion(z8_grados);

        uint8_t ids[] = {0, 1, 2,
        				3, 4, 5,
    					6, 7, 8,
    					9, 10, 11};

        uint16_t posiciones[] = {512, z3_posicion, z4_posicion,
        						512, z1_posicion, z2_posicion,
    							512, z5_posicion, z6_posicion,
    							512, z7_posicion, z8_posicion};


        syncwrite_mover_servos(ids, posiciones, 12);

        //HAL_Delay(0.001);
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 1000000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
