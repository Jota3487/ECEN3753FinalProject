///*
// * ApplicationCode.c
// *
// *  Created on: Nov 14, 2023
// *      Author: xcowa
// */
//
//#include "ApplicationCode.h"
//
//typedef struct {
//  uint8_t speed;
//  uint8_t increment_count;
//  uint8_t decrement_count;
//} SPEED_SETPOINT_DATA;
//
//typedef struct {
//  int direction;
//  uint8_t left_count;
//  uint8_t right_count;
//} VEHICLE_DIRECTION_DATA;
//
//int btn_speedup = 0;
//SPEED_SETPOINT_DATA speedData;
//VEHICLE_DIRECTION_DATA directionData;
//
//static osThreadId_t Lab7_SPEED_SETPOINT_TASK_ID;
//static osThreadId_t Lab7_VEHICLE_DIRECTION_TASK_ID;
//static osThreadId_t Lab7_VEHICLE_MONITOR_TASK_ID;
//static osThreadId_t Lab7_LED_OUTPUT_TASK_ID;
//static osThreadId_t Lab7_LCD_DISPLAY_TASK_ID;
//
//static osEventFlagsId_t Lab7_EventFlagID;
//static osSemaphoreId_t Lab7_Btn_SemaphoreID;
//static osSemaphoreId_t Lab7_Gyro_SemaphoreID;
//static osSemaphoreId_t Lab7_LCD_SemaphoreID;
//static osTimerId_t Lab7_Btn_Timer_ID;
//static osTimerId_t Lab7_Gyro_Timer_ID;
//
//static StaticTimer_t Lab7_Btn_Timer_CB; /**< Static callback memory for Lab3 Timer. */
//static const osTimerAttr_t Btn_Timer_Attributes = {
//	.name = "Lab7_Btn_Timer", /**< Name of the timer. */
//	.attr_bits = 0, /**< Attribute bits. */
//	.cb_mem = &Lab7_Btn_Timer_CB, /**< Control block memory. */
//	.cb_size = sizeof(Lab7_Btn_Timer_CB) /**< Memory size for control block. */
//};
//
//static StaticTimer_t Lab7_Gyro_Timer_CB; /**< Static callback memory for Lab3 Timer. */
//static const osTimerAttr_t Gyro_Timer_Attributes = {
//	.name = "Lab7_Gyro_Timer", /**< Name of the timer. */
//	.attr_bits = 0, /**< Attribute bits. */
//	.cb_mem = &Lab7_Gyro_Timer_CB, /**< Control block memory. */
//	.cb_size = sizeof(Lab7_Gyro_Timer_CB) /**< Memory size for control block. */
//};
//
//static osMutexId_t Lab7_Speed_Setpoint_Mutex_ID;
//static osMutexId_t Lab7_Vehicle_Direction_Mutex_ID;
//
//
///**
// * @brief Initializes the application.
// *
// * This function initializes various components required for the application to run.
// * It initializes the LTCD, LCD layer, Gyroscope, GPIO pins for buttons, and the RTOS.
// *
// * @note This function should be called before starting the application.
// */
//void ApplicationInit(void)
//{
//	LTCD__Init();
//    LTCD_Layer_Init(0);
//    Gyro_Init();
//
//	HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_RESET);
//
//	Lab7_RTOS_Init();
//}
//
///**
// * @brief Initializes the RTOS tasks, semaphores, mutexes, and timers for Lab 7.
// *
// * This function initializes various RTOS components required for Lab 7 operation,
// * including speed and direction data initialization, event flags, semaphores,
// * timers, mutexes, and task creation.
// *
// * @note This function should be called before starting the Lab 7 application.
// */
//void Lab7_RTOS_Init(void) {
//
//	speedData.speed = 30;
//	speedData.increment_count = 0;
//	speedData.decrement_count = 0;
//
//	directionData.direction = neutral;
//	directionData.left_count = 0;
//	directionData.right_count = 0;
//
//	Lab7_EventFlagID = osEventFlagsNew(NULL);
//
//	Lab7_Btn_SemaphoreID = osSemaphoreNew(1, 0, NULL);
//	Lab7_Gyro_SemaphoreID = osSemaphoreNew(1, 0, NULL);
//	Lab7_LCD_SemaphoreID = osSemaphoreNew(1, 0, NULL);
//
//	Lab7_Btn_Timer_ID = osTimerNew(Change_Acceleration, osTimerOnce, NULL, &Btn_Timer_Attributes);
//	Lab7_Gyro_Timer_ID = osTimerNew(Gyro_Direction_Post, osTimerPeriodic, NULL, &Gyro_Timer_Attributes);
//	if (Lab7_Gyro_Timer_ID != NULL)  {
//	osStatus_t status = osTimerStart(Lab7_Gyro_Timer_ID, 100u);       // start timer
//		if (status != osOK) {
//			// Timer could not be started
//			while(1){};
//		}
//	}
//
//	Lab7_Speed_Setpoint_Mutex_ID = osMutexNew(NULL);
//	Lab7_Vehicle_Direction_Mutex_ID = osMutexNew(NULL);
//
//	Lab7_SPEED_SETPOINT_TASK_ID = osThreadNew(Speed_Setpoint_Task, NULL, &Speed_Setpoint_Attributes);
//	if (Lab7_SPEED_SETPOINT_TASK_ID == NULL)  {
//		while(1){};
//	}
//
//	Lab7_VEHICLE_DIRECTION_TASK_ID = osThreadNew(Vehicle_Direction_Task, NULL, &Vehicle_Direction_Attributes);
//	if (Lab7_VEHICLE_DIRECTION_TASK_ID == NULL)  {
//		while(1){};
//	}
//
//	Lab7_VEHICLE_MONITOR_TASK_ID = osThreadNew(Vehicle_Monitor_Task, NULL, &Vehicle_Monitor_Attributes);
//	if (Lab7_VEHICLE_MONITOR_TASK_ID == NULL)  {
//		while(1){};
//	}
//	Lab7_LED_OUTPUT_TASK_ID = osThreadNew(LED_Output_Task, NULL, &LED_output_Attributes);
//	if (Lab7_LED_OUTPUT_TASK_ID == NULL)  {
//		while(1){};
//	}
//
//	Lab7_LCD_DISPLAY_TASK_ID = osThreadNew(LCD_Display_Task, NULL, &LCD_Display_Attributes);
//	if (Lab7_LCD_DISPLAY_TASK_ID == NULL)  {
//		while(1){};
//	}
//}
//
///**
// * @brief Changes acceleration based on button input.
// *
// * This function is called by a timer and sets a flag indicating a task action
// * should be taken. It is typically called in response to a button press event.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
//void Change_Acceleration(void *arg) {
//	(void)&arg;
//	btn_speedup = 1;
//}
//
///**
// * @brief Posts signals to semaphores after gyroscope direction reading.
// *
// * This function is called periodically by a timer to release semaphores,
// * allowing other tasks to proceed, particularly tasks related to gyroscope
// * data processing and LCD display updating.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
//void Gyro_Direction_Post(void *arg){
//	(void)&arg;
//	osSemaphoreRelease(Lab7_Gyro_SemaphoreID);
//	osSemaphoreRelease(Lab7_LCD_SemaphoreID);
//}
//
///**
// * @brief Reads the state of the user button.
// *
// * This function reads the state of the user button (typically a push button)
// * connected to a specific GPIO pin and returns the state as an integer.
// *
// * @return 1 if the user button is pressed, 0 otherwise.
// */
//int read_user_button_state(void){
//	if(HAL_GPIO_ReadPin(USER_BUTTON_PORT, USER_BUTTON_PIN)){
//		return 1;
//	}
//	return 0;
//}
//
///**
// * @brief Determines the rotation rate based on gyro velocity.
// *
// * This function determines the rotation rate based on the provided gyro velocity.
// * It categorizes the velocity into different rotation rates, such as fast clockwise,
// * slow clockwise, zero rotation, slow counterclockwise, and fast counterclockwise.
// *
// * @param gyro_vel The gyro velocity to be categorized.
// * @return The rotation rate based on the gyro velocity.
// */
//int get_gyro_rotation_rate(int16_t gyro_vel){
//	if(gyro_vel > FAST_CW) {
//		return FAST_CLOCKWISE;
//	} else if (gyro_vel > SLOW_CW) {
//		return SLOW_CLOCKWISE;
//	} else if(gyro_vel < SLOW_CW && gyro_vel > SLOW_CCW){
//		return ZERO_ROTATION;
//	} else if (gyro_vel < SLOW_CCW && gyro_vel > FAST_CCW){
//		return SLOW_COUNTERCLOCKWISE;
//	} else {
//		return FAST_COUNTERCLOCKWISE;
//	}
//}
//
///**
// * @brief Determines the direction based on gyro rotation rate.
// *
// * This function determines the direction based on the provided gyro rotation rate.
// * It categorizes the rotation rate into different directions, such as hard right,
// * right, neutral, left, and hard left.
// *
// * @param rotation_rate The gyro rotation rate to be categorized.
// * @return The direction based on the gyro rotation rate.
// */
//int get_gyro_direction(int rotation_rate){
//	if (rotation_rate == FAST_CLOCKWISE) {return hard_right;}
//	if (rotation_rate == SLOW_CLOCKWISE) {return right;}
//	if (rotation_rate == ZERO_ROTATION) {return neutral;}
//	if (rotation_rate == SLOW_COUNTERCLOCKWISE) {return left;}
//	if (rotation_rate == FAST_COUNTERCLOCKWISE) {return hard_left;}
//	return neutral;
//}
//
///**
// * @brief Runs a demo on the LCD.
// *
// * clears the LCD screen to white and runs a quick demo on it.
// * The demo typically involves displaying various graphical elements or
// * showcasing certain features of the LCD.
// */
//void RunDemoForLCD(void)
//{
//	LCD_Clear(0,LCD_COLOR_WHITE);
//	QuickDemo();
//}
//
///**
// * @brief GPIO External Interrupt Callback.
// *
// * This function is called when a GPIO external interrupt occurs.
// * It releases the semaphore associated with button handling in the Lab 7 application,
// * allowing the corresponding task to proceed.
// *
// * @param GPIO_Pin The GPIO pin number that triggered the interrupt.
// */
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
//{
//	osSemaphoreRelease(Lab7_Btn_SemaphoreID);
//}
//
///**
// * @brief Task for adjusting speed setpoint.
// *
// * This task continuously monitors the user button state to adjust the speed setpoint.
// * It waits for the button semaphore to be released, indicating a button press event.
// * Upon button press, it either increases or decreases the speed setpoint based on
// * the button state. It then updates the speed data and signals the event flag for
// * speed update.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
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
//
///**
// * @brief Task for determining vehicle direction.
// *
// * This task continuously monitors the gyroscope semaphore to acquire gyroscope data.
// * Upon acquiring the semaphore, it retrieves the gyroscope rotation rate and determines
// * the vehicle direction based on the rotation rate. It then updates the direction data
// * and signals the event flag for direction update.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
//void Vehicle_Direction_Task(void *arg){
//	while(1){
//		(void)&arg;
//		osStatus_t semaphoreStatus;
//		semaphoreStatus = osSemaphoreAcquire(Lab7_Gyro_SemaphoreID, osWaitForever);
//		if (semaphoreStatus == osOK) {
//			int rotation = get_gyro_rotation_rate(Gyro_Get_Velocity());
//			int direction = get_gyro_direction(rotation);
//			osStatus_t result = osMutexAcquire(Lab7_Vehicle_Direction_Mutex_ID, osWaitForever);
//			directionData.direction = direction;
//			if(direction < neutral) {
//				directionData.left_count++;
//				directionData.right_count = 0;
//			} else if(direction > neutral) {
//				directionData.right_count++;
//				directionData.left_count = 0;;
//			} else {
//				directionData.right_count = 0;
//				directionData.left_count = 0;;
//			}
//			osMutexRelease(Lab7_Vehicle_Direction_Mutex_ID);
//			osEventFlagsSet(Lab7_EventFlagID, DIRECTION_UPDATE_FLAG);
//		}
//	}
//}
//
///**
// * @brief Task for monitoring vehicle parameters.
// *
// * This task continuously monitors vehicle parameters such as speed and direction.
// * It waits for event flags indicating updates in speed or direction data.
// * Upon receiving these updates, it retrieves the latest speed and direction values
// * and checks if they exceed certain thresholds. Depending on the thresholds exceeded,
// * it sets corresponding event flags for LED alerts.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
//void Vehicle_Monitor_Task(void *arg){
//	uint8_t current_speed = 30;
//	int current_direction = 0;
//	uint8_t left_num = 0;
//	uint8_t right_num = 0;
//
//	while(1){
//		(void)&arg;
//		uint32_t data_flags;
//		data_flags = osEventFlagsWait(Lab7_EventFlagID, 0x00000003U, osFlagsWaitAny, osWaitForever);
//		//speed
//		if((data_flags & 1) == 1) {
//			osStatus_t speed_mutex = osMutexAcquire(Lab7_Speed_Setpoint_Mutex_ID, osWaitForever);  // lock count is incremented, might fail when lock count is depleted
//			if (speed_mutex == osOK) {
//				current_speed = speedData.speed;
//			}
//			osMutexRelease(Lab7_Speed_Setpoint_Mutex_ID);
//		}
//		//direction
//		if(((data_flags >> 1) & 1) == 1) {
//			osStatus_t direction_mutex = osMutexAcquire(Lab7_Vehicle_Direction_Mutex_ID, osWaitForever);
//			if (direction_mutex == osOK) {
//				current_direction = directionData.direction;
//				left_num = directionData.left_count;
//				right_num = directionData.right_count;
//			}
//			osMutexRelease(Lab7_Vehicle_Direction_Mutex_ID);
//		}
//		if((current_speed > 75) || (current_speed > 45 && (current_direction != neutral))){
//			osEventFlagsSet(Lab7_EventFlagID, GREEN_LED_ALERT_FLAG);
//		} else {
//			osEventFlagsSet(Lab7_EventFlagID, GREEN_LED_SAFE_FLAG);
//		}
//		if(left_num >= 50 || right_num >= 50){
//			osEventFlagsSet(Lab7_EventFlagID, RED_LED_ALERT_FLAG);
//		} else {
//			osEventFlagsSet(Lab7_EventFlagID, RED_LED_SAFE_FLAG);
//		}
//	}
//}
//
///**
// * @brief Task for controlling LED outputs.
// *
// * This task continuously monitors event flags related to LED control.
// * Upon receiving these event flags, it adjusts the states of the green and red LEDs
// * based on the flags received.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
//void LED_Output_Task(void *arg){
//	while(1){
//		(void)&arg;
//		uint32_t LED_flags;
//		LED_flags = osEventFlagsWait(Lab7_EventFlagID, 0x0000003CU, osFlagsWaitAny, osWaitForever);
//		if(((LED_flags >> 2) & 1) == 1) {
//			HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_SET);
//		}
//		if(((LED_flags >> 3) & 1) == 1) {
//			HAL_GPIO_WritePin(GREEN_BUTTON_PORT, GREEN_BUTTON_PIN, GPIO_PIN_RESET);
//		}
//		if(((LED_flags >> 4) & 1) == 1) {
//			HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_SET);
//		}
//		if(((LED_flags >> 5) & 1) == 1) {
//			HAL_GPIO_WritePin(RED_BUTTON_PORT, RED_BUTTON_PIN, GPIO_PIN_RESET);
//		}
//	}
//}
//
///**
// * @brief Task for displaying data on the LCD.
// *
// * This task continuously monitors the LCD semaphore to acquire access to the LCD display.
// * Upon acquiring the semaphore, it retrieves the latest speed and direction data.
// * It then updates the LCD display with the current speed and direction information.
// *
// * @param arg Pointer to additional argument (unused in this function).
// */
//void LCD_Display_Task(void *arg){
//	uint8_t current_speed = 30;
//	int current_direction = 0;
//	while(1){
//		(void)&arg;
//		osStatus_t status;
//		status = osSemaphoreAcquire(Lab7_LCD_SemaphoreID, osWaitForever);
//		if (status == osOK) {
//			osStatus_t speed_mutex = osMutexAcquire(Lab7_Speed_Setpoint_Mutex_ID, osWaitForever);  // lock count is incremented, might fail when lock count is depleted
//			if (speed_mutex == osOK) {
//				current_speed = speedData.speed;
//			}
//			osMutexRelease(Lab7_Speed_Setpoint_Mutex_ID);
//
//			osStatus_t direction_mutex = osMutexAcquire(Lab7_Vehicle_Direction_Mutex_ID, osWaitForever);
//			if (direction_mutex == osOK) {
//				current_direction = directionData.direction;
//			}
//			osMutexRelease(Lab7_Vehicle_Direction_Mutex_ID);
//
//			LCD_Clear(0, LCD_COLOR_WHITE);
//
//			LCD_SetTextColor(LCD_COLOR_BLACK);
//			LCD_SetFont(&Font16x24);
//
//			LCD_DisplayString(30,110,"Speed:");
//			LCD_DisplayString(30,200,"Direction:");
//			//send number to display
//			LCD_DisplayNumber(150,110,current_speed);
//
//			if (current_direction == 0) {LCD_DisplayString(30,230,"Hard left");}
//			if (current_direction == 1) {LCD_DisplayString(30,230,"left");}
//			if (current_direction == 2) {LCD_DisplayString(30,230,"neutral");}
//			if (current_direction == 3) {LCD_DisplayString(30,230,"right");}
//			if (current_direction == 4) {LCD_DisplayString(30,230,"Hard right");}
//		}
//	}
//}
