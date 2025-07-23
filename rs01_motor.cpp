#include "rs01_motor.h"

// Global variables
HardwareSerial* motor_serial = nullptr;
MI_Motor motors[2];
MotorDataCallback data_callback = NULL;

// Helper function to convert float to uint for CAN transmission
int float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Helper function to convert uint to float from CAN reception
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + offset;
}

/**
 * @brief Sends a raw 12-byte CAN frame (4-byte ID + 8-byte data) over UART.
 * @param frame Pointer to the can_frame_t structure.
 */
void UART_Send_Frame(const can_frame_t* frame) {
    if (!motor_serial) return;

    // Construct the 29-bit extended CAN ID directly from the frame structure
    uint32_t extended_id = ((uint32_t)frame->type << 24) |
                           ((uint32_t)frame->data << 8) |
                           frame->target_id;

    uint8_t packet[CAN_RAW_FRAME_LENGTH];

    // Pack the 4-byte extended CAN ID (big-endian)
    packet[0] = (extended_id >> 24) & 0xFF;
    packet[1] = (extended_id >> 16) & 0xFF;
    packet[2] = (extended_id >> 8) & 0xFF;
    packet[3] = extended_id & 0xFF;

    // Copy the 8-byte data payload
    memcpy(&packet[4], frame->payload, 8);

    // Uncomment to print TX data for debugging
    // Serial.print("TX: ");
    // for (size_t i = 0; i < sizeof(packet); i++) {
    //     if (packet[i] < 16) Serial.print("0");
    //     Serial.print(packet[i], HEX);
    //     Serial.print(" ");
    // }
    // Serial.println();

    motor_serial->write(packet, sizeof(packet));
}

/**
 * @brief Parses a received 12-byte CAN frame from the serial module.
 * @param can_payload Pointer to the 12-byte array.
 */
static void parse_can_frame(const uint8_t* can_payload) {
    uint32_t extended_id = ((uint32_t)can_payload[0] << 24) |
                           ((uint32_t)can_payload[1] << 16) |
                           ((uint32_t)can_payload[2] << 8) |
                           can_payload[3];

    uint8_t type = (extended_id >> 24) & 0x1F;

    // We are interested in feedback frames (Type 2) and auto-report frames (Type 24)
    if (type == 0x02 || type == 0x18) {
        uint8_t motor_id = (extended_id >> 8) & 0xFF;
        
        // Check if the motor ID is one we are controlling
        if (motor_id >= MOTER_1_ID && motor_id <= MOTER_2_ID) {
            MI_Motor* motor = &motors[motor_id - 1]; // Use motor_id - 1 for array index
            motor->id = motor_id; // Explicitly assign the parsed motor ID
            const uint8_t* data = &can_payload[4];


            // Parse data payload according to Type 2 format
            uint16_t raw_angle = (data[0] << 8) | data[1];
            uint16_t raw_speed = (data[2] << 8) | data[3];
            uint16_t raw_torque = (data[4] << 8) | data[5];
            uint16_t raw_temp = (data[6] << 8) | data[7];

            // Convert raw data to physical units
            motor->position = uint_to_float(raw_angle, P_MIN, P_MAX, 16);
            motor->velocity = uint_to_float(raw_speed, V_MIN, V_MAX, 16);
            motor->current = uint_to_float(raw_torque, T_MIN, T_MAX, 16);
            motor->temperature = raw_temp / 10.0f;
            
            // --- Detailed Status Parsing ---
            uint32_t status_part = extended_id >> 16;
            
            // Parse combined error code
            motor->error = status_part & 0x3F; // Bits 16-21

            // Parse individual error flags

            motor->error_undervoltage      = (status_part >> 0) & 0x01; // Bit 16
            motor->error_driver_fault      = (status_part >> 1) & 0x01; // Bit 17
            motor->error_over_temperature  = (status_part >> 2) & 0x01; // Bit 18
            motor->error_magnetic_encoder  = (status_part >> 3) & 0x01; // Bit 19
            motor->error_overload          = (status_part >> 4) & 0x01; // Bit 20
            motor->error_uncalibrated      = (status_part >> 5) & 0x01; // Bit 21

            // Parse mode status
            motor->mode = (status_part >> 6) & 0x03; // Bits 22-23

            // --- Print Parsed Data for Debugging ---
            Serial.printf("  [Parsed] ID: %d, Pos: %.2f, Vel: %.2f, Cur: %.2f, Temp: %.1f, Mode: %d, Err: 0x%X\n",
                          motor->id, motor->position, motor->velocity, motor->current, motor->temperature, motor->mode, motor->error);

            if (data_callback) {
                data_callback(motor);
            }
        }
    }
}

/**
 * @brief Handles incoming UART bytes.
 */
void handle_uart_rx() {
    if (!motor_serial) return;

    static uint8_t packet_buffer[CAN_RAW_FRAME_LENGTH];
    static uint8_t packet_index = 0;

    while (motor_serial->available()) {
        if (packet_index < CAN_RAW_FRAME_LENGTH) {
            packet_buffer[packet_index++] = motor_serial->read();
        } else {
            motor_serial->read(); 
            packet_index = 0;
        }

        if (packet_index >= CAN_RAW_FRAME_LENGTH) {
            // Serial.print("RX: ");
            // for (int i = 0; i < CAN_RAW_FRAME_LENGTH; i++) {
            //     if (packet_buffer[i] < 16) Serial.print("0");
            //     Serial.print(packet_buffer[i], HEX);
            //     Serial.print(" ");
            // }
            // Serial.println();

            parse_can_frame(packet_buffer);
            packet_index = 0;
        }
    }
}

/**
 * @brief Initializes UART communication.
 * @param callback Function pointer for data updates.
 */
void UART_Rx_Init(MotorDataCallback callback) {
    motor_serial = &Serial1;
    motor_serial->begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
    data_callback = callback;
}

// --- Motor Control Functions ---

void Motor_Enable(MI_Motor* motor) {
    can_frame_t frame;
    frame.type = 0x03;
    frame.target_id = motor->id;
    frame.data = MASTER_ID; // Per doc, data field (Bit23~8) is master ID
    memset(frame.payload, 0, sizeof(frame.payload)); // Data payload is all zeros for enable command
    UART_Send_Frame(&frame);
}

void Motor_Reset(MI_Motor* motor, uint8_t clear_error) {
    can_frame_t frame;
    frame.type = 0x04;
    frame.target_id = motor->id;
    frame.data = MASTER_ID;
    memset(frame.payload, 0, sizeof(frame.payload));
    if (clear_error) {
        frame.payload[0] = 1;
    }
    UART_Send_Frame(&frame);
}

void Motor_Set_Zero(MI_Motor* motor) {
    can_frame_t frame;
    frame.type = 0x06;
    frame.target_id = motor->id;
    frame.data = MASTER_ID;
    memset(frame.payload, 0, sizeof(frame.payload));
    frame.payload[0] = 1;
    UART_Send_Frame(&frame);
}

void Motor_ControlMode(MI_Motor* motor, float torque, float position, float speed, float kp, float kd) {
    can_frame_t frame;
    frame.type = 0x01;
    frame.target_id = motor->id;
    frame.data = float_to_uint(torque, T_MIN, T_MAX, 16);

    uint16_t pos_uint = float_to_uint(position, P_MIN, P_MAX, 16);
    uint16_t spd_uint = float_to_uint(speed, V_MIN, V_MAX, 16);
    uint16_t kp_uint = float_to_uint(kp, KP_MIN, KP_MAX, 16);
    uint16_t kd_uint = float_to_uint(kd, KD_MIN, KD_MAX, 16);

    frame.payload[0] = pos_uint >> 8;
    frame.payload[1] = pos_uint & 0xFF;
    frame.payload[2] = spd_uint >> 8;
    frame.payload[3] = spd_uint & 0xFF;
    frame.payload[4] = kp_uint >> 8;
    frame.payload[5] = kp_uint & 0xFF;
    frame.payload[6] = kd_uint >> 8;
    frame.payload[7] = kd_uint & 0xFF;

    UART_Send_Frame(&frame);
}

void Set_SingleParameter(MI_Motor* motor, uint16_t parameter_index, float value) {
    can_frame_t frame;
    frame.type = 0x12; // Type 18
    frame.target_id = motor->id;
    frame.data = MASTER_ID;

    memset(frame.payload, 0, sizeof(frame.payload));
    
    // Parameter index (little-endian in payload)
    frame.payload[0] = parameter_index & 0xFF;
    frame.payload[1] = (parameter_index >> 8) & 0xFF;

    // Parameter value (float, little-endian)
    memcpy(&frame.payload[4], &value, sizeof(float));

    UART_Send_Frame(&frame);
}

void Set_CurMode(MI_Motor* motor, float current) {
    Set_SingleParameter(motor, IQ_REF, current);
}

void Change_Mode(MI_Motor* motor, uint8_t mode) {
    can_frame_t frame;
    frame.type = 0x12; // Type 18
    frame.target_id = motor->id;
    frame.data = MASTER_ID;

    memset(frame.payload, 0, sizeof(frame.payload));
    
    uint16_t index = RUN_MODE;
    // Parameter index (little-endian in payload)
    frame.payload[0] = index & 0xFF;
    frame.payload[1] = (index >> 8) & 0xFF;

    // Mode value
    frame.payload[4] = mode;

    UART_Send_Frame(&frame);
}
