/*
 * ApplicationCode.c
 *
 *  Created on: Nov 14, 2023
 *      Author: Jonathan Tang
 */

#include "ApplicationCode.h"


// Define an enum for PinAtCenter
typedef enum {
    DRONE,
    MAZE
} PinType;

typedef struct {
    float Gravity;                  // [kg*cm/(s^2)]
    float UpdateFrequency;            // [Hz]
    int PinAtCenter;            // [enum:0=DRONE, 1=MAZE]
    int AngleGain;                  // [-/1000]
} Physics;

typedef struct {
	struct {
		int isActive;
		int ActiveTime;
        int MaxTime;                // [ms]
        int Power;                  // [mW]
        int MinActivationEnergy;    // [mJ]
    } Disruptor;

    struct {
        int MaxEnergy;              // [mJ]
        int EnergyLeft;
        int isRecharging;
        int isDischarging;
        int RechargeRate;           // [mW]
    } EnergyStore;

    int Diameter;                   // [mm]
    double X_Pos;
    double Y_Pos;
} Drone;


typedef struct {
    int TimeToComplete;         // [ms]
    int CellSize;               // [mm]
    int reset;
    struct {
        int Width;              // [cells]
        int Height;             // [cells]
    } Size;
    struct {
        int Wall;               // [Pr*1000]
        int Hole;               // [Pr*1000]
    } ObstacleProbability;
    int HoleDiameter;           // [mm]
    int HardEdged;              // [bool]
    struct {
        int Number;             // [-]
        int Diameter;           // [mm]
        int Reuse;              // [bool]
        struct {
            int x;              // [mm]
            int y;              // [mm]
            int hit;
        } Location[4];          // (x,y pairs) [mm]
    } Waypoints;
} Maze;

Physics physics;
Drone drone;
Maze maze;

int16_t x_gyro_value = 0;
int16_t y_gyro_value = 0;
double x_angle = 0;
double y_angle = 0;
double velX = 0;               // cm/s
double velY = 0;                 		// cm/s
int matrix_wall[MAZE_WIDTH][MAZE_HEIGHT][2]; //0:right 1:bottom
int matrix_hole[MAZE_WIDTH][MAZE_HEIGHT];

int hole_hit = 0;
int btn_count = 0;

//SPEED_SETPOINT_DATA speedData;
//VEHICLE_DIRECTION_DATA directionData;

//static osThreadId_t Lab7_SPEED_SETPOINT_TASK_ID;
static osThreadId_t Gyro_Task_ID;
//static osThreadId_t Lab7_VEHICLE_MONITOR_TASK_ID;
static osThreadId_t LED_GREEN_TASK_ID;
static osThreadId_t LED_RED_TASK_ID;
static osThreadId_t LCD_DISPLAY_TASK_ID;

static osEventFlagsId_t Lab7_EventFlagID;
static osSemaphoreId_t Btn_SemaphoreID;
static osSemaphoreId_t Gyro_SemaphoreID;
static osSemaphoreId_t LCD_SemaphoreID;
static osTimerId_t Lab7_Btn_Timer_ID;
static osTimerId_t Gyro_Timer_ID;
static osTimerId_t LCD_Timer_ID;
static osTimerId_t Life_Timer_ID;
static osTimerId_t Recharge_Timer_ID;
static osTimerId_t Discharge_Timer_ID;


static StaticTimer_t Lab7_Btn_Timer_CB; /**< Static callback memory for Lab3 Timer. */
static const osTimerAttr_t Btn_Timer_Attributes = {
	.name = "Lab7_Btn_Timer", /**< Name of the timer. */
	.attr_bits = 0, /**< Attribute bits. */
	.cb_mem = &Lab7_Btn_Timer_CB, /**< Control block memory. */
	.cb_size = sizeof(Lab7_Btn_Timer_CB) /**< Memory size for control block. */
};

static StaticTimer_t Gyro_Timer_CB; /**< Static callback memory for Lab3 Timer. */
static const osTimerAttr_t Gyro_Timer_Attributes = {
	.name = "Gyro_Timer", /**< Name of the timer. */
	.attr_bits = 0, /**< Attribute bits. */
	.cb_mem = &Gyro_Timer_CB, /**< Control block memory. */
	.cb_size = sizeof(Gyro_Timer_CB) /**< Memory size for control block. */
};

static StaticTimer_t LCD_Timer_CB; /**< Static callback memory for Lab3 Timer. */
static const osTimerAttr_t LCD_Timer_Attributes = {
	.name = "LCD_Timer", /**< Name of the timer. */
	.attr_bits = 0, /**< Attribute bits. */
	.cb_mem = &LCD_Timer_CB, /**< Control block memory. */
	.cb_size = sizeof(LCD_Timer_CB) /**< Memory size for control block. */
};

static StaticTimer_t Life_Timer_CB; /**< Static callback memory for Lab3 Timer. */
static const osTimerAttr_t Life_Timer_Attributes = {
	.name = "Life_Timer", /**< Name of the timer. */
	.attr_bits = 0, /**< Attribute bits. */
	.cb_mem = &Life_Timer_CB, /**< Control block memory. */
	.cb_size = sizeof(Life_Timer_CB) /**< Memory size for control block. */
};
static StaticTimer_t Recharge_Timer_CB; /**< Static callback memory for Lab3 Timer. */
static const osTimerAttr_t Recharge_Timer_Attributes = {
	.name = "Recharge_Timer", /**< Name of the timer. */
	.attr_bits = 0, /**< Attribute bits. */
	.cb_mem = &Recharge_Timer_CB, /**< Control block memory. */
	.cb_size = sizeof(Recharge_Timer_CB) /**< Memory size for control block. */
};

static StaticTimer_t Discharge_Timer_CB; /**< Static callback memory for Lab3 Timer. */
static const osTimerAttr_t Discharge_Timer_Attributes = {
	.name = "Discharge_Timer", /**< Name of the timer. */
	.attr_bits = 0, /**< Attribute bits. */
	.cb_mem = &Discharge_Timer_CB, /**< Control block memory. */
	.cb_size = sizeof(Discharge_Timer_CB) /**< Memory size for control block. */
};

static osMutexId_t Lab7_Speed_Setpoint_Mutex_ID;
static osMutexId_t Lab7_Vehicle_Direction_Mutex_ID;


/**
 * @brief Initializes the application.
 *
 * This function initializes various components required for the application to run.
 * It initializes the LTCD, LCD layer, Gyroscope, GPIO pins for buttons, and the RTOS.
 *
 * @note This function should be called before starting the application.
 */
void ApplicationInit(void)
{
	LTCD__Init();
    LTCD_Layer_Init(0);
    Gyro_Init();

	HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_RESET);

	Project_RTOS_Init();
}

/**
 * @brief Initializes the RTOS tasks, semaphores, mutexes, and timers for Lab 7.
 *
 * This function initializes various RTOS components required for Lab 7 operation,
 * including speed and direction data initialization, event flags, semaphores,
 * timers, mutexes, and task creation.
 *
 * @note This function should be called before starting the Lab 7 application.
 */
void Project_RTOS_Init(void) {

    // Create an instance of the Physics struct
	physics.Gravity = 9.8 * 5,
	physics.UpdateFrequency = 50,
	physics.PinAtCenter = DRONE,
	physics.AngleGain = 500;

    // Create an instance of the Drone struct
	drone.Disruptor.isActive = 0;
	drone.Disruptor.ActiveTime = 0;
	drone.Disruptor.MaxTime = 1000;
	drone.Disruptor.Power = 10000;
	drone.Disruptor.MinActivationEnergy = 6000;
	drone.EnergyStore.MaxEnergy = 15000;
	drone.EnergyStore.EnergyLeft = 1000;
	drone.EnergyStore.RechargeRate = 1000;
	drone.Diameter = 10;
    drone.X_Pos = 20;
    drone.Y_Pos = 20;


    // Create an instance of the Maze struct
	maze.TimeToComplete = 30000;
	maze.CellSize = 12;
	maze.reset = 0;
	maze.Size.Width = 15;
	maze.Size.Height = 15;
	maze.ObstacleProbability.Wall = 100,
	maze.ObstacleProbability.Hole = 200;
	maze.HoleDiameter = 11;
	maze.HardEdged = 1;  // TRUE
	maze.Waypoints.Number = 4;
	maze.Waypoints.Diameter = 90;
	maze.Waypoints.Reuse = 0;  // FALSE
	maze.Waypoints.Location[0].x = 50;   // (#0: starting point for game)
	maze.Waypoints.Location[0].y = 50;
	maze.Waypoints.Location[1].x = 130;
	maze.Waypoints.Location[1].y = 50;
	maze.Waypoints.Location[2].x = 50;
	maze.Waypoints.Location[2].y = 130;
	maze.Waypoints.Location[3].x = 130;
	maze.Waypoints.Location[3].y = 130;
// ------------------------------------------------------------------------------------


	Lab7_EventFlagID = osEventFlagsNew(NULL);

	Btn_SemaphoreID = osSemaphoreNew(1, 0, NULL);
	Gyro_SemaphoreID = osSemaphoreNew(1, 0, NULL);
	LCD_SemaphoreID = osSemaphoreNew(1, 0, NULL);

	Lab7_Btn_Timer_ID = osTimerNew(Change_Acceleration, osTimerOnce, NULL, &Btn_Timer_Attributes);
	Gyro_Timer_ID = osTimerNew(Gyro_Position_Post, osTimerPeriodic, NULL, &Gyro_Timer_Attributes);
	if (Gyro_Timer_ID != NULL)  {
	osStatus_t status = osTimerStart(Gyro_Timer_ID, 20u);       // start timer
		if (status != osOK) {
			// Timer could not be started
			while(1){};
		}
	}

	LCD_Timer_ID = osTimerNew(LCD_Post, osTimerPeriodic, NULL, &LCD_Timer_Attributes);
	if (LCD_Timer_ID != NULL)  {
	osStatus_t status = osTimerStart(LCD_Timer_ID, 100u);       // start timer
		if (status != osOK) {
			// Timer could not be started
			while(1){};
		}
	}

	Life_Timer_ID = osTimerNew(Game_Countdown, osTimerPeriodic, NULL, &Life_Timer_Attributes);
	if (Life_Timer_ID != NULL)  {
	osStatus_t status = osTimerStart(Life_Timer_ID, 1000u);       // start timer
		if (status != osOK) {
			// Timer could not be started
			while(1){};
		}
	}
	Recharge_Timer_ID = osTimerNew(Energy_Recharge, osTimerPeriodic, NULL, &Recharge_Timer_Attributes);
	Discharge_Timer_ID = osTimerNew(Energy_Discharge, osTimerPeriodic, NULL, &Discharge_Timer_Attributes);


	Lab7_Speed_Setpoint_Mutex_ID = osMutexNew(NULL);
	Lab7_Vehicle_Direction_Mutex_ID = osMutexNew(NULL);

//	Lab7_SPEED_SETPOINT_TASK_ID = osThreadNew(Speed_Setpoint_Task, NULL, &Speed_Setpoint_Attributes);
//	if (Lab7_SPEED_SETPOINT_TASK_ID == NULL)  {
//		while(1){};
//	}

	Gyro_Task_ID = osThreadNew(Gyro_Drone_Task, NULL, &Gyro_Attributes);
	if (Gyro_Task_ID == NULL)  {
		while(1){};
	}
//
//	Lab7_VEHICLE_MONITOR_TASK_ID = osThreadNew(Vehicle_Monitor_Task, NULL, &Vehicle_Monitor_Attributes);
//	if (Lab7_VEHICLE_MONITOR_TASK_ID == NULL)  {
//		while(1){};
//	}
	LED_GREEN_TASK_ID = osThreadNew(LED_Green_Task, NULL, &LED_Green_Attributes);
	if (LED_GREEN_TASK_ID == NULL)  {
		while(1){};
	}

	LED_RED_TASK_ID = osThreadNew(LED_Red_Task, NULL, &LED_Red_Attributes);
	if (LED_RED_TASK_ID == NULL)  {
		while(1){};
	}

	LCD_DISPLAY_TASK_ID = osThreadNew(LCD_Display_Task, NULL, &LCD_Display_Attributes);
	if (LCD_DISPLAY_TASK_ID == NULL)  {
		while(1){};
	}

	maze_init();
}

float mazeToLCD(float cell_pixel, float cell_min, float cell_max, float LCD_min, float LCD_max) {
    float transformed_pixel = (cell_pixel - cell_min) / (cell_max - cell_min);
    float LCD_pixel = LCD_min + (LCD_max - LCD_min) * transformed_pixel;
    return LCD_pixel;
}

void maze_init(void){
	srand(time(NULL));
	int rand_wall;
	int rand_hole;
	for(int i = 0; i < maze.Size.Width; i++){
		for(int j = 0; j < maze.Size.Width; j++) {
			rand_wall = rand() % 10;
			if(rand_wall < 1){
				matrix_wall[i][j][0] = 1;
			} else {
				matrix_wall[i][j][0] = 0;
			}
			rand_wall = rand() % 10;
			if(rand_wall < 1){
				matrix_wall[i][j][1] = 1;
			} else {
				matrix_wall[i][j][1] = 0;
			}
			if(i == 14) {
				matrix_wall[i][j][0] = 1;
			}
			if(j == 14) {
				matrix_wall[i][j][1] = 1;
			}
			rand_hole = rand() % 5;
			if(rand_hole < 1){
				matrix_hole[i][j] = 1;
			} else {
				matrix_hole[i][j] = 0;
			}
		}
	}
}

void Game_Countdown(void *arg) {
	(void)&arg;
	if(maze.TimeToComplete <= 0){
		Reset_Game();
	} else {
		if(!DEVELOPER_MODE) {
			maze.TimeToComplete -= 1000;
		}
	}
}


/**
 * @brief Changes acceleration based on button input.
 *
 * This function is called by a timer and sets a flag indicating a task action
 * should be taken. It is typically called in response to a button press event.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void Change_Acceleration(void *arg) {
	(void)&arg;
//	btn_speedup = 1;
}

/**
 * @brief Posts signals to semaphores after gyroscope direction reading.
 *
 * This function is called periodically by a timer to release semaphores,
 * allowing other tasks to proceed, particularly tasks related to gyroscope
 * data processing and LCD display updating.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void Gyro_Position_Post(void *arg){
	(void)&arg;
	osSemaphoreRelease(Gyro_SemaphoreID);
}

void LCD_Post(void *arg){
	(void)&arg;
	if(maze.reset == 0) {
		osSemaphoreRelease(LCD_SemaphoreID);
	}
}

/**
 * @brief Reads the state of the user button.
 *
 * This function reads the state of the user button (typically a push button)
 * connected to a specific GPIO pin and returns the state as an integer.
 *
 * @return 1 if the user button is pressed, 0 otherwise.
 */
int read_user_button_state(void){
	if(HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN)){
		return 1;
	}
	return 0;
}

/**
 * @brief Determines the rotation rate based on gyro velocity.
 *
 * This function determines the rotation rate based on the provided gyro velocity.
 * It categorizes the velocity into different rotation rates, such as fast clockwise,
 * slow clockwise, zero rotation, slow counterclockwise, and fast counterclockwise.
 *
 * @param gyro_vel The gyro velocity to be categorized.
 * @return The rotation rate based on the gyro velocity.
 */
int get_gyro_rotation_rate(int16_t gyro_vel){
	if(gyro_vel > FAST_CW) {
		return FAST_CLOCKWISE;
	} else if (gyro_vel > SLOW_CW) {
		return SLOW_CLOCKWISE;
	} else if(gyro_vel < SLOW_CW && gyro_vel > SLOW_CCW){
		return ZERO_ROTATION;
	} else if (gyro_vel < SLOW_CCW && gyro_vel > FAST_CCW){
		return SLOW_COUNTERCLOCKWISE;
	} else {
		return FAST_COUNTERCLOCKWISE;
	}
}

/**
 * @brief Determines the direction based on gyro rotation rate.
 *
 * This function determines the direction based on the provided gyro rotation rate.
 * It categorizes the rotation rate into different directions, such as hard right,
 * right, neutral, left, and hard left.
 *
 * @param rotation_rate The gyro rotation rate to be categorized.
 * @return The direction based on the gyro rotation rate.
 */
int get_gyro_direction(int rotation_rate){
	if (rotation_rate == FAST_CLOCKWISE) {return hard_right;}
	if (rotation_rate == SLOW_CLOCKWISE) {return right;}
	if (rotation_rate == ZERO_ROTATION) {return neutral;}
	if (rotation_rate == SLOW_COUNTERCLOCKWISE) {return left;}
	if (rotation_rate == FAST_COUNTERCLOCKWISE) {return hard_left;}
	return neutral;
}

/**
 * @brief Runs a demo on the LCD.
 *
 * clears the LCD screen to white and runs a quick demo on it.
 * The demo typically involves displaying various graphical elements or
 * showcasing certain features of the LCD.
 */
void RunDemoForLCD(void)
{
	//LCD_Clear(0,LCD_COLOR_WHITE);
	QuickDemo();
}

/**
 * @brief GPIO External Interrupt Callback.
 *
 * This function is called when a GPIO external interrupt occurs.
 * It releases the semaphore associated with button handling in the Lab 7 application,
 * allowing the corresponding task to proceed.
 *
 * @param GPIO_Pin The GPIO pin number that triggered the interrupt.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	osSemaphoreRelease(Btn_SemaphoreID);
	if(read_user_button_state()){
		if(drone.EnergyStore.EnergyLeft >= drone.Disruptor.MinActivationEnergy) {
			drone.Disruptor.isActive = 1;
			drone.EnergyStore.isRecharging = 0;
		}
	}
	else {
		drone.Disruptor.isActive = 0;
		drone.Disruptor.ActiveTime = 0;
		drone.EnergyStore.isDischarging = 0;
	}
}

void Energy_Recharge(void *arg)
{
	(void)&arg;
	if(drone.EnergyStore.EnergyLeft < drone.EnergyStore.MaxEnergy) {
		drone.EnergyStore.EnergyLeft += drone.EnergyStore.RechargeRate;
	}
}
void Energy_Discharge(void *arg)
{
	(void)&arg;
	if(drone.Disruptor.ActiveTime < 10 && drone.EnergyStore.EnergyLeft >= drone.Disruptor.Power/10) {
		if(!DEVELOPER_MODE){
			drone.EnergyStore.EnergyLeft -= drone.Disruptor.Power/10;
		}
	} else {
		if(DEVELOPER_MODE == 0) {
			drone.Disruptor.isActive = 0;
		}
	}
	drone.Disruptor.ActiveTime ++;
}
/**
 * @brief Task for adjusting speed setpoint.
 *
 * This task continuously monitors the user button state to adjust the speed setpoint.
 * It waits for the button semaphore to be released, indicating a button press event.
 * Upon button press, it either increases or decreases the speed setpoint based on
 * the button state. It then updates the speed data and signals the event flag for
 * speed update.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
//void Speed_Setpoint_Task(void *arg){
//	while(1){
//		(void)&arg;
//		osStatus_t semaphoreStatus;
//		osStatus_t timer_status;
//		semaphoreStatus = osSemaphoreAcquire(Lab7_Btn_SemaphoreID, osWaitForever);
//		if (semaphoreStatus == osOK) {
//			if(read_user_button_state()) {
//				btn_speedup = 0;
//				timer_status = osTimerStart(Lab7_Btn_Timer_ID, 1000u);
//			} else {
//				timer_status = osTimerStop(Lab7_Btn_Timer_ID);
//				if(!btn_speedup){
//					//speed up
//					osStatus_t mutex_result = osMutexAcquire(Lab7_Speed_Setpoint_Mutex_ID, osWaitForever);  // lock count is incremented, might fail when lock count is depleted
//					if (mutex_result == osOK) {
//						speedData.speed += 5;
//						speedData.increment_count++;
//					}
//					osMutexRelease(Lab7_Speed_Setpoint_Mutex_ID);
//				}else {
//					//slow down
//					osStatus_t mutex_result = osMutexAcquire(Lab7_Speed_Setpoint_Mutex_ID, osWaitForever);  // lock count is incremented, might fail when lock count is depleted
//					if (mutex_result == osOK) {
//						speedData.speed -= 5;
//						speedData.decrement_count++;
//					}
//					osMutexRelease(Lab7_Speed_Setpoint_Mutex_ID);
//				}
//				osEventFlagsSet(Lab7_EventFlagID, SPEED_UPDATE_FLAG);
//			}
//
//		}
//	}
//}

/**
 * @brief Task for determining vehicle direction.
 *
 * This task continuously monitors the gyroscope semaphore to acquire gyroscope data.
 * Upon acquiring the semaphore, it retrieves the gyroscope rotation rate and determines
 * the vehicle direction based on the rotation rate. It then updates the direction data
 * and signals the event flag for direction update.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
float x_acceleration = 0;
float y_acceleration = 0;

float previous_xPos = 20;
float previous_yPos = 20;
int test_x = 0;
int test_y = 0;

void Gyro_Drone_Task(void *arg){
	while(1){
		(void)&arg;
		double tau = 1/physics.UpdateFrequency;
		osStatus_t semaphoreStatus;
		semaphoreStatus = osSemaphoreAcquire(Gyro_SemaphoreID, osWaitForever);
		if (semaphoreStatus == osOK) {
//			x_angle += (float)17.5/(float)1000.0 * Gyro_Get_X_Velocity() * tau; //make sure not 0
//			y_angle += (float)17.5/(float)1000.0 * Gyro_Get_Y_Velocity() * tau;

//			x_gyro_value = Gyro_Get_X_Velocity();
//			y_gyro_value = Gyro_Get_Y_Velocity();

			test_x = 0;
			test_y = 0;
			x_angle += ((double)17.5/(double)1000.0 * (double)Gyro_Get_X_Velocity() * (double)tau); //make sure not 0
			y_angle += ((double)17.5/(double)1000.0 * (double)Gyro_Get_Y_Velocity() * (double)tau);

			if(fabs(x_angle) >= 90 ||  fabs(y_angle) >= 90) {
				Reset_Game();
			}


			  double x_acceleration = physics.Gravity * sin((double)x_angle*M_PI/(double)180);
			  double y_acceleration = physics.Gravity * sin((double)y_angle*M_PI/(double)180);

			  velX += (x_acceleration * tau);               // cm/s
			  velY += (y_acceleration * tau);               		// cm/s
			  drone.X_Pos += (velY * tau);               // cm
			  drone.Y_Pos += (velX * tau);                		    // cm

		  if (!drone.Disruptor.isActive) {
			  if(Check_Hole_Collision(drone.X_Pos, drone.Y_Pos, 4)){
//				  velX = 0;
//				  velY = 0;
				  hole_hit++; // Adjust
				  if(!DEVELOPER_MODE){
					  Reset_Game();
				  }
			  }


			if(Check_Circle_Collision(drone.X_Pos, drone.Y_Pos, 5)) {
			int c_x = Check_Circle_Collision_X(drone.X_Pos, drone.Y_Pos, 5);
			int c_y = Check_Circle_Collision_Y(drone.X_Pos, drone.Y_Pos, 5);
			if(abs(c_x - drone.X_Pos) >= 1.91) {
				velX = 0;
				drone.X_Pos = previous_xPos;
				test_x = 1;
			}
			if(abs(c_y - drone.Y_Pos) >= 1.91) {
				velY = 0;
				drone.Y_Pos = previous_yPos;
				test_y = 1;
			}
			}
		  }

		  if (!drone.Disruptor.isActive && !drone.EnergyStore.isRecharging){
			  osTimerStart(Recharge_Timer_ID, 1000u);
			  drone.EnergyStore.isRecharging = 1;
		  } else if (drone.Disruptor.isActive){
			  osTimerStop(Recharge_Timer_ID);
		  }


		  if(drone.Disruptor.isActive && !drone.EnergyStore.isDischarging){
			  osTimerStart(Discharge_Timer_ID, 100u);
			  drone.EnergyStore.isDischarging = 1;
		  }else if(!drone.Disruptor.isActive){
			  osTimerStop(Discharge_Timer_ID);
		  }

		  if(drone.X_Pos >= 225) {
			  drone.X_Pos = 225;
			  velX = 0;
		  }
		  if(drone.X_Pos <= 15) {
		  			  drone.X_Pos = 15;
		  			  velX = 0;
		  		  }
		  if(drone.Y_Pos >= 225) {
		  			  drone.Y_Pos = 225;
		  			  velY = 0;
		  		  }
		  if(drone.Y_Pos <= 15) {
		  			  drone.Y_Pos = 15;
		  			  velY = 0;
		  		  }
			  //LCD_DisplayNumber(100, 130, y_acceleration);
			//LCD_Draw_Circle_Fill(drone.X_Pos, drone.Y_Pos, 3, LCD_COLOR_BLACK);
			//osMutexRelease(Gyro_SemaphoreID);
			//osEventFlagsSet(Lab7_EventFlagID, DIRECTION_UPDATE_FLAG);

		  if(Check_Waypoint_Collision(drone.X_Pos, drone.Y_Pos, 5)) {
			  if(drone.X_Pos < 110 && drone.Y_Pos < 110){
				  maze.Waypoints.Location[0].hit = 1;
			  } else if (drone.X_Pos > 110 && drone.Y_Pos < 110){
				  maze.Waypoints.Location[1].hit = 1;
			  }else if (drone.X_Pos < 110 && drone.Y_Pos > 110){
				  maze.Waypoints.Location[2].hit = 1;
			  }else if (drone.X_Pos > 110 && drone.Y_Pos > 110){
				  maze.Waypoints.Location[3].hit = 1;
			  }
		  }

		  if(maze.Waypoints.Location[0].hit && maze.Waypoints.Location[1].hit && maze.Waypoints.Location[2].hit && maze.Waypoints.Location[3].hit){
			  Win_Game();
		  }

		    previous_xPos = drone.X_Pos;
		    previous_yPos = drone.Y_Pos;
		}
	}
}


/**
 * @brief Task for controlling LED outputs.
 *
 * This task continuously monitors event flags related to LED control.
 * Upon receiving these event flags, it adjusts the states of the green and red LEDs
 * based on the flags received.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void LED_Green_Task(void *arg){
	while(1){
		(void)&arg;
		if(drone.EnergyStore.EnergyLeft == 0) {
			HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_RESET);
			osDelay(50);
		} else if(drone.EnergyStore.EnergyLeft > 0 && drone.EnergyStore.EnergyLeft < 10000){
			HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_SET);
			osDelay(drone.EnergyStore.EnergyLeft/1000);
			HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_RESET);
			osDelay(10 - (drone.EnergyStore.EnergyLeft/1000));
		}else {
			HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_SET);
			osDelay(1000);
		}
	}
}

void LED_Red_Task(void *arg){
	while(1){
		(void)&arg;
		if(drone.EnergyStore.EnergyLeft < drone.Disruptor.MinActivationEnergy) {
			float period =(drone.Disruptor.MinActivationEnergy - drone.EnergyStore.EnergyLeft)/1000;
			HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_SET);
			osDelay(500 / period);
			HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_RESET);
			osDelay(500 / period);
		} else {
			HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_RESET);
			osDelay(1000);
		}
	}
}

void Reset_Game(void) {

	maze.reset = 1;
	LCD_Clear(0, LCD_COLOR_MAZE);
	LCD_DisplayString(50, 100, "You Lost");
	LCD_DisplayString(50, 130, "Try Again");
	osDelay(3000);

	drone.Disruptor.isActive = 0;
	drone.Disruptor.ActiveTime = 0;
	drone.EnergyStore.EnergyLeft = 1000;
	drone.X_Pos = 20;
	drone.Y_Pos = 20;
	maze.TimeToComplete = 30000;
	maze.Waypoints.Location[0].hit = 0;
	maze.Waypoints.Location[1].hit = 0;
	maze.Waypoints.Location[2].hit = 0;
	maze.Waypoints.Location[3].hit = 0;

	maze.reset = 0;

}

void Win_Game(void) {

	maze.reset = 1;
	LCD_Clear(0, LCD_COLOR_CYAN);
	LCD_DisplayString(50, 100, "You Win !!");
	LCD_DisplayString(15, 130, "Bing Chilling!!");
	osDelay(3000);

	drone.Disruptor.isActive = 0;
	drone.Disruptor.ActiveTime = 0;
	drone.EnergyStore.EnergyLeft = 1000;
	drone.X_Pos = 20;
	drone.Y_Pos = 20;
	maze.TimeToComplete = 30000;
	maze.Waypoints.Location[0].hit = 0;
	maze.Waypoints.Location[1].hit = 0;
	maze.Waypoints.Location[2].hit = 0;
	maze.Waypoints.Location[3].hit = 0;

	maze.reset = 0;

}


void maze_top_left (void) {
    for (int i = 0; i < 15; i++) {
    	LCD_Draw_Vertical_Line(mazeToLCD (0, 0, maze.CellSize * maze.Size.Width, 10, 229), mazeToLCD(i*12, 0, maze.CellSize * maze.Size.Width, 10, 229), LCD_CELL_LINE, LCD_COLOR_BLACK);
    	LCD_Draw_Horizontal_Line(mazeToLCD (i*12, 0, maze.CellSize * maze.Size.Width, 10, 229), mazeToLCD(0, 0, maze.CellSize * maze.Size.Width, 10, 229), LCD_CELL_LINE, LCD_COLOR_BLACK);
//    	LCD_DisplayNumber(100, 20*i, mazeToLCD(i*12, 0, maze.CellSize * maze.Size.Width, 10, 229));
    }
}

void draw_right_wall(float x, float y){
	float LCD_X = mazeToLCD ((x+1)*maze.CellSize, 0, 180, 10, 229);
	float LCD_Y = mazeToLCD (y*maze.CellSize, 0, 180, 10, 229);
	LCD_Draw_Vertical_Line(LCD_X, LCD_Y, LCD_CELL_LINE, LCD_COLOR_BLACK);
}
void draw_bottom_wall(float x, float y){
	float LCD_X = mazeToLCD (x*maze.CellSize, 0, 180, 10, 229);
	float LCD_Y = mazeToLCD ((y+1)*maze.CellSize, 0, 180, 10, 229);
	LCD_Draw_Horizontal_Line(LCD_X, LCD_Y, LCD_CELL_LINE, LCD_COLOR_BLACK);
}
void draw_hole(float x, float y){
	float LCD_X = mazeToLCD ((x+0.5)*maze.CellSize, 0, 180, 10, 229);
	float LCD_Y = mazeToLCD ((y+0.5)*maze.CellSize, 0, 180, 10, 229);
	LCD_Draw_Circle_Fill(LCD_X, LCD_Y, 5.5, LCD_COLOR_RED);
}

/**
 * @brief Task for displaying data on the LCD.
 *
 * This task continuously monitors the LCD semaphore to acquire access to the LCD display.
 * Upon acquiring the semaphore, it retrieves the latest speed and direction data.
 * It then updates the LCD display with the current speed and direction information.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void LCD_Display_Task(void *arg){
	while(1){
		osStatus_t semaphoreStatus;
		semaphoreStatus = osSemaphoreAcquire(LCD_SemaphoreID, osWaitForever);

			LCD_Clear(0, LCD_COLOR_MAZE);

			LCD_SetTextColor(LCD_COLOR_BLACK);
			LCD_SetFont(&Font16x24);

			if(maze.Waypoints.Location[0].hit == 0) {
				LCD_Draw_Circle_Fill(mazeToLCD(maze.Waypoints.Location[0].x, 0, 180, 10, 229), mazeToLCD (maze.Waypoints.Location[0].y, 0, 180, 10, 229), 45*LCD_CELL_LINE/12, LCD_COLOR_YELLOW);
			}
			if(maze.Waypoints.Location[1].hit == 0) {
				LCD_Draw_Circle_Fill(mazeToLCD(maze.Waypoints.Location[1].x, 0, 180, 10, 229), mazeToLCD (maze.Waypoints.Location[1].y, 0, 180, 10, 229), 45*LCD_CELL_LINE/12, LCD_COLOR_YELLOW);
			}
			if(maze.Waypoints.Location[2].hit == 0) {
				LCD_Draw_Circle_Fill(mazeToLCD(maze.Waypoints.Location[2].x, 0, 180, 10, 229), mazeToLCD (maze.Waypoints.Location[2].y, 0, 180, 10, 229), 45*LCD_CELL_LINE/12, LCD_COLOR_YELLOW);
			}
			if(maze.Waypoints.Location[3].hit == 0) {
				LCD_Draw_Circle_Fill(mazeToLCD(maze.Waypoints.Location[3].x, 0, 180, 10, 229), mazeToLCD (maze.Waypoints.Location[3].y, 0, 180, 10, 229), 45*LCD_CELL_LINE/12, LCD_COLOR_YELLOW);
			}

        maze_top_left();
    	for(int i = 0; i < maze.Size.Width; i++){
    		for(int j = 0; j < maze.Size.Width; j++) {
    			if(matrix_wall[i][j][0]) {
    				draw_right_wall(i,j);
    			}
    			if(matrix_wall[i][j][1]) {
    				draw_bottom_wall(i,j);
    			}
    			if(matrix_hole[i][j]) {
    				draw_hole(i,j);
				}
    		}
    	}

        char angle_x_str[20];
        char angle_y_str[20];


        //Display angle values on the LCD
        LCD_DisplayString(10, 250, angle_x_str);
        LCD_DisplayString(120, 250, angle_y_str);

        sprintf(angle_x_str, "aX:%d", (int)x_angle);
        sprintf(angle_y_str, "aY:%d", (int)y_angle);
//        sprintf(period_str, "%d", 500*period);

//        LCD_DisplayString(150, 290, period_str);


        LCD_DisplayNumber(10, 280, maze.TimeToComplete/1000);
        LCD_DisplayNumber(210, 290, hole_hit);
        LCD_DisplayNumber(210, 270, drone.Disruptor.isActive);
        LCD_DisplayNumber(120, 270, drone.EnergyStore.EnergyLeft);


        if(test_x == 1){LCD_DisplayString(20,232,"x");}else{LCD_DisplayString(20,232,"_");}
        if(test_y == 1){LCD_DisplayString(100,232,"y");}else{LCD_DisplayString(100,232,"_");}
        LCD_Draw_Circle_Fill(drone.X_Pos, drone.Y_Pos, 5, LCD_COLOR_BLUE); // Adjust
	}
}
