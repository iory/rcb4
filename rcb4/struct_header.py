import cstruct

max_SIO_num = 2
max_sensor_num = 24
sensor_sidx = 38
max_wormmodule_num = max_sensor_num

ServoStruct = cstruct.parse("""
struct ServoStruct
{
  uint8_t flag; // 1: available
  uint8_t mode; //  servo EEPROM address 14,15
  uint8_t rotation; // 1:rotation servo, 0:hinge servo
  uint8_t port; // 1:J1:huart2, 4:J2:huart7,
                // 6:J3:huart5, 2:J4:huart4, 3:J5:huart3, 5:J6:huart1
  //
  uint16_t ref_angle;
  uint16_t current_angle;
  int16_t trim;
  uint16_t start_ref_angle;
  uint16_t cmd_ref_angle;
  uint16_t prev_angle;
  //
  uint8_t params[64];
  //
  uint8_t frame;
  uint8_t stretch;
  uint8_t speed;
  uint8_t current_limit;
  uint8_t temperature_limit;
  // eeprom
  uint8_t eeprom_update;
  uint8_t timeout_count;
  uint8_t rxerror_count;
  uint32_t count_frame;
  uint32_t prev_frame;

  uint8_t init_state; //0:no connection 1:initialization_state 2:initialized
  uint8_t init_count;
  int16_t error_angle;

  float p_gain;    // for rotation servo
  float i_gain;    // for rotation servo
  float d_gain;    // for rotation servo
  float e_sum;    // for rotation servo
  uint8_t feedback;  // for rotation servo 1: on 0:off
  uint8_t move_state; // 0:stop or interpolated, 1:moving
  uint8_t current;    // servo current CW:0 - 63, CCW:64 -127
  uint8_t temperature;  // servo temperature: 1 - 127
};
""")
ServoStruct.__name__ = "servo_vector"


SensorbaseStruct = cstruct.parse("""
#define SC_NONE 0x40
#define SC_SW_ADC 0x41
#define SC_PS  0x42
#define SC_IMU 0x44
#define SC_MAGENC 0x48
#define SC_ALL (SC_SW_ADC | SC_PS | SC_IMU | SC_MAGENC)
#define SC_SW_ADC_R (SC_SW_ADC & 0x0F)
#define SC_PS_R (SC_PS & 0x0F)
#define SC_IMU_R (SC_IMU & 0x0F)
#define SC_MAGENC_R (SC_MAGENC & 0x0F)
#define ADC_CHANNEL_NUM 5
#define FORCE_CHANNEL_NUM 4
#define PS_CHANNEL_NUM 4
#define GYRO_CHANNEL_NUM 3
#define ACC_CHANNEL_NUM 3
#define MAGENC_CHANNEL_NUM 1
#define ADC_LEN ADC_CHANNEL_NUM * 2
#define PS_LEN PS_CHANNEL_NUM * 3
#define GYRO_LEN GYRO_CHANNEL_NUM * 3
#define ACC_LEN ACC_CHANNEL_NUM * 3
#define IMU_LEN GYRO_LEN + ACC_LEN
#define MAGENC_LEN MAGENC_CHANNEL_NUM * 2
#define MAX_BUFFER_LENGTH 2 + ADC_LEN + PS_LEN + IMU_LEN + MAGENC_LEN

struct SensorbaseStruct
{
    uint8_t port;
    uint8_t id;
    uint8_t mode;
    uint8_t sw;
    //uint16_t adc[ADC_CHANNEL_NUM]; TODO:for KJS voltage measurement
    uint16_t adc[FORCE_CHANNEL_NUM];//adc for force sensor
    uint16_t ps[PS_CHANNEL_NUM];
    int16_t gyro[GYRO_CHANNEL_NUM];
    int16_t acc[ACC_CHANNEL_NUM];
    uint16_t rxerror_count;
    uint16_t timeout_count;
    uint8_t debug_rxbuff[2];
    uint32_t count_frame;//us
    uint32_t prev_frame;//us
    uint16_t adc_calib[(FORCE_CHANNEL_NUM + 1) * 5];//for FSR calibration
    float force[FORCE_CHANNEL_NUM];//caliculated from adc val
    uint16_t temperature;
    uint16_t magenc;

    uint16_t adc_offset[FORCE_CHANNEL_NUM];//initial adc value
    float adc_calib_hsf[4 * (FORCE_CHANNEL_NUM + 1)];
    // for HSFPAR003A force calibration
    // [bFz,aFz0~aFz3],[bMx,aMx0~aMx3],[bMy,aMy0~aMy3],
    // [Fz scale[N -> g], Mx scale[Nm -> Nmm], My scale[Nm -> Nmm],
    // calib_paramA, calib_paramB]
    float wrench[3];//[Fz [g],Mx [Nmm],My[Nmm]]
    float wrench_offset[3];//[Fz [g],Mx [Nmm],My[Nmm]]
    float wrench_scale[3];//[Fz [g],Mx [Nmm],My[Nmm]]
    uint8_t wrench_init_count;
    uint8_t wrench_init_done;
    uint8_t sensor_updated;
    uint8_t is_contact;
    uint8_t board_revision;
    uint8_t move_state;
    uint8_t dummy[2];
};
""")
SensorbaseStruct.__name__ = "Sensor_vector"


GPIOStruct = cstruct.parse("""
struct GPIOStruct
{
    uint8_t port;
    uint8_t id;
    uint8_t mode;
    uint8_t gpio_updated;
};
""")
GPIOStruct.__name__ = "GPIO_vector"


ImuData = cstruct.parse("""
struct ImuData_t
{
    int16_t acc[3];
    int16_t gyro[3];
    int16_t mag[3];
}
""")
ImuData.__name__ = "imu_data_"


Madgwick = cstruct.parse("""
struct Madgwick {
  float beta;				// algorithm gain
  float q0;
  float q1;
  float q2;
  float q3;	// quaternion of sensor frame relative to auxiliary frame
  float SampleFreq;
  float invSampleFreq;
  float roll;
  float pitch;
  float yaw;
  float gyro_scale;
  float acc[3];
  float gyro[3];
  float gyro_offset[3];
  float mag[3];
  uint32_t calib_cnt;
  uint32_t calib_flag;
  uint32_t anglesComputed;
  float gyro_norm;
  float gyro_norm_thleshold;
}
""")
Madgwick.__name__ = "Mfilter"


WormmoduleStruct = cstruct.parse("""
struct WormmoduleStruct
{
  uint8_t module_type;//r/w 0:MODULE_NONE, 1:MODULE_ROTATE, 2:MODULE_LINEAR
  uint8_t servo_id;//r/w same ID as rcb4robotconfig defined
  uint8_t sensor_id;//r/w raw ID,, TODO: change reference
  uint8_t move_state;//as same as servo
  uint16_t magenc_init;
  uint16_t magenc_present;
  uint16_t magenc_ref;
  uint16_t thleshold;//r magenc value scale
  float ref_angle;//angle scale
  float linear_upper_limit;//upper limit for linear module(ms)
  float magenc_error;
  float present_angle;//angle scale
  float thleshold_scale;
  float target_time;//ms
  float timeout_time;//ms
  float timeout_time_scale;
  float estimated_speed;
  float gear_ratio;
  float move_time;//ms
  float count_time;//ms
  uint32_t start_frame;//ns
  uint32_t prev_frame;//ns
  uint8_t overflow_times;
  uint8_t dummy[3];
}; //1*4 + 2*4 + 4*14 + 4 = 72byte
""")
WormmoduleStruct.__name__ = "Worm_vector"


SystemStruct = cstruct.parse("""
struct SystemStruct
{
  uint32_t cur_time;
  uint32_t prev_time;
  float dt;
  float prev_dt;
  // The type of rom_flash and rom_memory has been changed from uint8_t* to uint32_t.
  // This is because pointers are 32 bits length in the STM32 chip in the kondoh7 board.
  uint32_t rom_flash; // FLASH area for RCB4 ROM area requires 256KB
  uint32_t rom_memory; // FLASH area for RCB4 ROM area requires 256KB
  uint16_t ad_value;
  uint16_t servo_cnt;
  float battery_voltage;
  float battery_voltage_vref;
  float vref;
  float initial_battery_voltage;
  float battery_voltage_percent;
  uint8_t battery_cal_done;
  uint8_t servo_current_read_flag;
  uint8_t servo_temperature_read_flag;
  uint8_t scan_mode;//0:normal, 1:simple
  uint16_t temperature;
  uint16_t buzzer_hz;
  uint8_t battery_type;
  uint8_t board_revision;
  uint8_t ics_baudrate[6];
  uint8_t ics_comm_stop[6]; // for handshake 6 UARTs param setting
  uint32_t frame_time; // used for ICS comm cycle in uarttasks.c
};
""")
SystemStruct.__name__ = "SysB"


DataAddress = cstruct.parse("""
struct DataAddress
{
  uint32_t _sidata;
  uint32_t _sdata;
  uint32_t _edata;
  uint32_t data_size;
  uint32_t dataflash_address;
  uint32_t src_addr;
  uint32_t dst_addr;
  uint32_t copy_size;
};
""")
DataAddress.__name__ = "data_address"


ControlStruct = cstruct.parse("""
#define RTOS_TASK_NUM 16
#define MAX_SIO_NUM 2 // default of kondoh7(2023/08/25) refer to rcb4 controlboard uart port num
#define MAX_DEVICE_NUM_PER_PORT 32
struct ControlStruct {
  uint32_t move_time;   // ms
  uint32_t count_time;  // ms
  uint32_t start_frame; // ns
  uint32_t prev_frame;  // ns
  uint8_t active;
  uint8_t prev_active;
  uint8_t servo_state; // 0:free 1:hold
  uint8_t buffc;
  int pc;
  uint8_t counter;
  uint8_t motion_flag;
  uint8_t interpolation;
  uint8_t len;
  uint32_t call_stack[16];
  uint8_t stack_index;
  uint8_t state;
  uint8_t prev_state;
  uint8_t dummy_pad_short_boundary;
  uint32_t rtos_maxstack[RTOS_TASK_NUM];
  uint32_t pprev_frame[6];
  uint32_t pcount_frame[6];
  uint16_t comm_params[8];
  float krs3304r2_kv; //[rpm/v]
  uint32_t taskprev_frame[RTOS_TASK_NUM];
  uint32_t taskcount_frame[RTOS_TASK_NUM];
  uint8_t device_port[MAX_SIO_NUM * MAX_DEVICE_NUM_PER_PORT];
  uint8_t device_timeout_th;
  uint8_t dummy[3];
};
""")
ControlStruct.__name__ = "Control"

c_vector = {
    "servo_vector": 36,
    "Sensor_vector": max_sensor_num,
    "imu_data_": 0,
    "Mfilter": 1,
    "Worm_vector": max_wormmodule_num,
    "SysB": 0,
    "data_address": 0,
    "Control": 0,
}
