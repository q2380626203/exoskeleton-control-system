#ifndef _MI_MOTOR_H__
#define _MI_MOTOR_H__

#include <stdint.h>
#include <string.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <cmath>

// UART Configuration
#define MOTOR_SERIAL_PORT    Serial1    // Hardware serial port for RS01 motor communication
#define UART_TX_PIN          13         // UART transmit pin  
#define UART_RX_PIN          12         // UART receive pin  
#define UART_BAUDRATE        115200     // UART baud rate

// CAN-to-Serial Module Protocol
#define CAN_RAW_FRAME_LENGTH 12 

// Motor and Master IDs
#define MASTER_ID        0xFD
#define MOTER_1_ID       1
#define MOTER_2_ID       2

// Motor Parameter Ranges (from RS01 manual)
#define P_MIN           -12.57f 
#define P_MAX           12.57f 
#define V_MIN           -44.0f 
#define V_MAX           44.0f 
#define KP_MIN          0.0f 
#define KP_MAX          500.0f 
#define KD_MIN          0.0f 
#define KD_MAX          5.0f 
#define T_MIN           -17.0f
#define T_MAX           17.0f

// Control Mode and Parameter Indices (from RS01 manual)
#define RUN_MODE        0x7005    // Run Mode: 0-Motion, 1-Position(PP), 2-Speed, 3-Current, 5-Position(CSP)
#define CTRL_MODE       0         // Motion Control Mode
#define POS_MODE_PP     1         // Position Mode (PP)
#define SPEED_MODE      2         // Speed Mode
#define CUR_MODE        3         // Current Mode
#define POS_MODE_CSP    5         // Position Mode (CSP)

#define IQ_REF          0x7006    // Current Mode Iq Command (float, -23~23A)
#define SPD_REF         0x700A    // Speed Mode Speed Command (float, -44~44rad/s)
#define LIMIT_TORQUE    0x700B    // Torque Limit (float, 0~17Nm)
#define LOC_REF         0x7016    // Position Mode Angle Command (float, rad)
#define LIMIT_SPD       0x7017    // Position Mode (CSP) Speed Limit (float, 0~44rad/s)
#define LIMIT_CUR       0x7018    // Speed/Position Mode Current Limit (float, 0~23A)

// Structure for outgoing CAN frames (based on RS01 protocol)
typedef struct {
    uint8_t type;       // Communication type (5 bits)
    uint16_t data;      // Data field (16 bits)
    uint8_t target_id;  // Target motor ID (8 bits)
    uint8_t payload[8]; // 8-byte data payload
} can_frame_t;

// Structure for local motor status
typedef struct {
    uint8_t id;         // Motor ID
    float position;     // Current position feedback
    float velocity;     // Current velocity feedback
    float current;      // Current feedback (interpreted from torque data)
    float temperature;  // Temperature feedback
    
    // Detailed status from feedback frame
    uint8_t mode;       // 0:Reset, 1:Cali, 2:Motor
    uint8_t error;      // Combined error code (Bit 16-21)

    // Individual error flags
    bool error_uncalibrated;
    bool error_overload;
    bool error_magnetic_encoder;
    bool error_over_temperature;
    bool error_driver_fault;
    bool error_undervoltage;

} MI_Motor;

// Callback function type for motor data updates
typedef void (*MotorDataCallback)(MI_Motor*);

// Global variables for UART communication.
extern HardwareSerial* motor_serial;
extern MI_Motor motors[2];
extern MotorDataCallback data_callback;

// Function Prototypes
void UART_Rx_Init(MotorDataCallback callback);
void handle_uart_rx();
void UART_Send_Frame(const can_frame_t* frame);

void Motor_Enable(MI_Motor* motor);
void Motor_Reset(MI_Motor* motor, uint8_t clear_error);
void Motor_Set_Zero(MI_Motor* motor);
void Motor_ControlMode(MI_Motor* motor, float torque, float position, float speed, float kp, float kd);
void Set_SingleParameter(MI_Motor* motor, uint16_t parameter, float value);
void Set_CurMode(MI_Motor* motor, float current);
void Change_Mode(MI_Motor* motor, uint8_t mode);

// Helper function for data conversion
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif // _MI_MOTOR_H__
