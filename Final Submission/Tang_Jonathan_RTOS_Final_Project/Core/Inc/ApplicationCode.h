/*
 * ApplicationCode.h
 *
 *  Created on: Nov 14, 2023
 *      Author: xcowa
 */

#ifndef INC_APPLICATIONCODE_H_
#define INC_APPLICATIONCODE_H_

#include "LCD_Driver.h"
#include "Gyro_Driver.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "main.h"
#include "math.h"
#include "time.h"
#include "stdlib.h"

#define DEVELOPER_MODE	1

#define USER_BUTTON_PORT	GPIOA
#define USER_BUTTON_PIN     GPIO_PIN_0

#define GREEN_BUTTON_PORT	GPIOG
#define GREEN_BUTTON_PIN    GPIO_PIN_13

#define RED_BUTTON_PORT		GPIOG
#define RED_BUTTON_PIN     	GPIO_PIN_14

#define FAST_CW			4000
#define SLOW_CW			400
#define	STATIC_RATE		0
#define SLOW_CCW		-400
#define FAST_CCW		-4000

#define MAZE_WIDTH 			15
#define MAZE_HEIGHT 		15

typedef enum
{
	GREEN_LED_FLAG = 0x00000001U,
	RED_LED_FLAG = 0x00000002U,
}FLAG_STATE;

typedef enum
{
  FAST_COUNTERCLOCKWISE = 0,
  SLOW_COUNTERCLOCKWISE = 1,
  ZERO_ROTATION = 2,
  SLOW_CLOCKWISE = 3,
  FAST_CLOCKWISE = 4,
}GYRO_RANGE;

typedef enum
{
  hard_left,
  left,
  neutral,
  right,
  hard_right,
}DIRECTION_RANGE;



static StaticTask_t Lab7_SPEED_SETPOINT_TASK_CB;
static uint32_t Lab7_SPEED_SETPOINT_TASK_Stack[256];
static const osThreadAttr_t Speed_Setpoint_Attributes = {
	.name = "Lab7_SPEED_SETPOINT_TASK", /**< Name of the thread. */
	.cb_mem = &Lab7_SPEED_SETPOINT_TASK_CB, /**< Control block memory. */
	.cb_size = sizeof(Lab7_SPEED_SETPOINT_TASK_CB), /**< Memory size for control block. */
	.stack_mem = &Lab7_SPEED_SETPOINT_TASK_Stack, /**< Stack memory location. */
	.stack_size = sizeof(Lab7_SPEED_SETPOINT_TASK_Stack), /**< Size of stack memory. */
	.priority = osPriorityNormal /**< Priority of the thread. */
};

static StaticTask_t Gyro_TASK_CB;
static uint32_t Gyro_TASK_Stack[256];
static const osThreadAttr_t Gyro_Attributes = {
	.name = "Gyro_TASK", /**< Name of the thread. */
	.cb_mem = &Gyro_TASK_CB, /**< Control block memory. */
	.cb_size = sizeof(Gyro_TASK_CB), /**< Memory size for control block. */
	.stack_mem = &Gyro_TASK_Stack, /**< Stack memory location. */
	.stack_size = sizeof(Gyro_TASK_Stack), /**< Size of stack memory. */
	.priority = osPriorityNormal /**< Priority of the thread. */
};

static StaticTask_t Lab7_VEHICLE_MONITOR_TASK_CB;
static uint32_t Lab7_VEHICLE_MONITOR_TASK_Stack[256];
static const osThreadAttr_t Vehicle_Monitor_Attributes = {
	.name = "Lab7_VEHICLE_MONITOR_TASK", /**< Name of the thread. */
	.cb_mem = &Lab7_VEHICLE_MONITOR_TASK_CB, /**< Control block memory. */
	.cb_size = sizeof(Lab7_VEHICLE_MONITOR_TASK_CB), /**< Memory size for control block. */
	.stack_mem = &Lab7_VEHICLE_MONITOR_TASK_Stack, /**< Stack memory location. */
	.stack_size = sizeof(Lab7_VEHICLE_MONITOR_TASK_Stack), /**< Size of stack memory. */
	.priority = osPriorityNormal /**< Priority of the thread. */
};

static StaticTask_t LED_GREEN_TASK_CB;
static uint32_t LED_GREEN_TASK_Stack[256];
static const osThreadAttr_t LED_Green_Attributes = {
	.name = "LED_GREEN_TASK", /**< Name of the thread. */
	.cb_mem = &LED_GREEN_TASK_CB, /**< Control block memory. */
	.cb_size = sizeof(LED_GREEN_TASK_CB), /**< Memory size for control block. */
	.stack_mem = &LED_GREEN_TASK_Stack, /**< Stack memory location. */
	.stack_size = sizeof(LED_GREEN_TASK_Stack), /**< Size of stack memory. */
	.priority = osPriorityNormal /**< Priority of the thread. */
};

static StaticTask_t LED_RED_TASK_CB;
static uint32_t LED_RED_TASK_Stack[256];
static const osThreadAttr_t LED_Red_Attributes = {
	.name = "LED_RED_TASK", /**< Name of the thread. */
	.cb_mem = &LED_RED_TASK_CB, /**< Control block memory. */
	.cb_size = sizeof(LED_RED_TASK_CB), /**< Memory size for control block. */
	.stack_mem = &LED_RED_TASK_Stack, /**< Stack memory location. */
	.stack_size = sizeof(LED_RED_TASK_Stack), /**< Size of stack memory. */
	.priority = osPriorityNormal /**< Priority of the thread. */
};

static StaticTask_t LCD_DISPLAY_TASK_CB;
static uint32_t LCD_DISPLAY_TASK_Stack[256];
static const osThreadAttr_t LCD_Display_Attributes = {
	.name = "LCD_DISPLAY_TASK", /**< Name of the thread. */
	.cb_mem = &LCD_DISPLAY_TASK_CB, /**< Control block memory. */
	.cb_size = sizeof(LCD_DISPLAY_TASK_CB), /**< Memory size for control block. */
	.stack_mem = &LCD_DISPLAY_TASK_Stack, /**< Stack memory location. */
	.stack_size = sizeof(LCD_DISPLAY_TASK_Stack), /**< Size of stack memory. */
	.priority = osPriorityNormal /**< Priority of the thread. */
};

/**
 * @brief Initializes the application.
 *
 * This function initializes various components required for the application to run.
 * It initializes the LTCD, LCD layer, Gyroscope, GPIO pins for buttons, and the RTOS.
 *
 * @note This function should be called before starting the application.
 */

void ApplicationInit(void);
/**
 * @brief Initializes the RTOS tasks, semaphores, mutexes, and timers for Lab 7.
 *
 * This function initializes various RTOS components required for Lab 7 operation,
 * including speed and direction data initialization, event flags, semaphores,
 * timers, mutexes, and task creation.
 *
 * @note This function should be called before starting the Lab 7 application.
 */

void Project_RTOS_Init(void);
void maze_init(void);
void Game_Countdown(void *arg);
/**
 * @brief Changes acceleration based on button input.
 *
 * This function is called by a timer and sets a flag indicating a task action
 * should be taken. It is typically called in response to a button press event.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void Change_Acceleration(void *arg);

/**
 * @brief Posts signals to semaphores after gyroscope direction reading.
 *
 * This function is called periodically by a timer to release semaphores,
 * allowing other tasks to proceed, particularly tasks related to gyroscope
 * data processing and LCD display updating.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void Gyro_Position_Post(void *arg);

/**
 * @brief Reads the state of the user button.
 *
 * This function reads the state of the user button (typically a push button)
 * connected to a specific GPIO pin and returns the state as an integer.
 *
 * @return 1 if the user button is pressed, 0 otherwise.
 */
int read_user_button_state(void);

void Energy_Recharge(void *arg);
void Energy_Discharge(void *arg);

void LCD_Post(void *arg);

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
int get_gyro_rotation_rate(int16_t gyro_vel);
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
int get_gyro_direction(int rotation_rate);

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
void Speed_Setpoint_Task(void *arg);

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
void Gyro_Drone_Task(void *arg);

/**
 * @brief Task for monitoring vehicle parameters.
 *
 * This task continuously monitors vehicle parameters such as speed and direction.
 * It waits for event flags indicating updates in speed or direction data.
 * Upon receiving these updates, it retrieves the latest speed and direction values
 * and checks if they exceed certain thresholds. Depending on the thresholds exceeded,
 * it sets corresponding event flags for LED alerts.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void Vehicle_Monitor_Task(void *arg);

/**
 * @brief Task for controlling LED outputs.
 *
 * This task continuously monitors event flags related to LED control.
 * Upon receiving these event flags, it adjusts the states of the green and red LEDs
 * based on the flags received.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void LED_Green_Task(void *arg);
void LED_Red_Task(void *arg);

void Reset_Game(void);
void Win_Game(void);
/**
 * @brief Task for displaying data on the LCD.
 *
 * This task continuously monitors the LCD semaphore to acquire access to the LCD display.
 * Upon acquiring the semaphore, it retrieves the latest speed and direction data.
 * It then updates the LCD display with the current speed and direction information.
 *
 * @param arg Pointer to additional argument (unused in this function).
 */
void LCD_Display_Task(void *arg);
void RunDemoForLCD(void);

#endif /* INC_APPLICATIONCODE_H_ */
