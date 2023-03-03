#include <SPI.h>
#include <DueTimer.h>
#include "Wire.h"
#include "bmi2.h"
#include "bmi270.h"

/*! I2C interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_I2C UINT8_C(0)

/*! SPI interface communication, 1 - Enable; 0- Disable */
#define BMI270_INTERFACE_SPI UINT8_C(1)

static int8_t bmi2_set_config(struct bmi2_dev *bmi2_dev);
#if BMI270_INTERFACE_I2C == 1
/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t i2c_bus;
#elif BMI270_INTERFACE_SPI == 1
static uint8_t spi_bus;
#endif

#define BMI270_CS       10

/* Create an instance of sensor data structure */
struct bmi2_sensor_data sensor_data = { 0 };

/* Structure to define BMI2 sensor configurations */
struct bmi2_dev bmi2;

int8_t BMI270_write_spi(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  uint32_t cnt;
  int8_t rev = 0;
  (void)(intf_ptr);
  digitalWrite(BMI270_CS, LOW);
  SPI.transfer(reg_addr);
  for (cnt = 0; cnt < length; cnt++)
  {
    SPI.transfer(*(reg_data + cnt));
  }
  digitalWrite(BMI270_CS, HIGH);
  return rev;
}

int8_t BMI270_read_spi(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  uint32_t cnt;
  int8_t rev = 0;
  (void)(intf_ptr);
  reg_addr = 0x80 | reg_addr;
  digitalWrite(BMI270_CS, LOW);
  SPI.transfer(reg_addr);
  for (cnt = 0; cnt < length; cnt++)
  {
    *(reg_data + cnt) = SPI.transfer(0x00);
  }
  digitalWrite(BMI270_CS, HIGH);
  return rev;
}


void bmi2xy_hal_delay_usec(uint32_t period_us, void *intf_ptr)
{
  delayMicroseconds(period_us);
}

/*! This API is used to perform I2C read operation with sensor */
int8_t bmi2xy_hal_i2c_bus_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //uint8_t dev_id = 0x68;
  uint8_t* dev_id = (uint8_t *)intf_ptr;

  rslt = BMI270_read_i2c(*dev_id, reg_addr, reg_data, length);

  return rslt;
}

/*! This API is used to perform I2C write operations with sensor */
int8_t bmi2xy_hal_i2c_bus_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
  int8_t rslt = 0;
  //    uint8_t dev_id = 0x68;

  uint8_t* dev_id = (uint8_t *)intf_ptr;
  rslt = BMI270_write_i2c(*dev_id, reg_addr, (uint8_t *)reg_data, length);

  return rslt;
}

int8_t BMI270_read_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /* dev_addr: I2C device address.
    reg_addr: Starting address for writing the data.
    reg_data: Data to be written.
    count: Number of bytes to write */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();

  if (returned)
  {
    /*
      case 1:Data too long to fit in transmit buffer
          break;
      case 2:received NACK on transmit of address.
          break;
      case 3:received NACK on transmit of data."
          break;
      case 4:Unspecified error.
          break;
      default:Unexpected Wire.endTransmission() return code:
    */
    return returned;
  }

  // Requests the required number of bytes from the sensor
  Wire.requestFrom((int)dev_addr, (int)count);

  uint16_t i;
  // Reads the requested number of bytes into the provided array
  for (i = 0; (i < count) && Wire.available(); i++)
  {
    reg_data[i] = Wire.read(); // This is for the modern Wire library
  }

  // This must return 0 on success, any other value will be interpreted as a communication failure.
  return 0;
}

int8_t BMI270_write_i2c(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t count)
{
  /*  dev_addr: I2C device address.
    reg_addr: Starting address for reading the data.
    reg_data: Buffer to take up the read data.
    count: Number of bytes to read. */
  // Begin I2C communication with provided I2C address
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);

  uint16_t i;
  // Writes the requested number of bytes from the provided array
  for (i = 0; i < count; i++)
  {
    Wire.write(reg_data[i]); // This is for the modern Wire library
  }
  // Done writting, end the transmission
  int8_t returned = Wire.endTransmission();
  /*
      case 1:Data too long to fit in transmit buffer
      case 2:received NACK on transmit of address.
      case 3:received NACK on transmit of data.
      case 4:Unspecified error.
      default:Unexpected Wire.endTransmission() return code:
  */
  // This must return 0 on sucess, any other value will be interpretted as a communication failure.
  return returned;
}

int8_t bmi2_accel_set_config(struct bmi2_dev *bmi2_dev)
{
  /* Variable to define result */
  int8_t rslt;

  /* Initialize interrupts for gyroscope */
  uint8_t sens_int = BMI2_DRDY_INT;

  /* List the sensors which are required to enable */
  uint8_t sens_list = BMI2_ACCEL;

  /* Structure to define the type of the sensor and its configurations */
  struct bmi2_sens_config config;

  /* Configure type of feature */
  config.type = BMI2_ACCEL;

  /* Enable the selected sensors */
  rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);

  if (rslt == BMI2_OK)
  {
    /* Get default configurations for the type of feature selected */
    rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {

      config.cfg.acc.odr = BMI2_ACC_ODR_100HZ;

      /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
      config.cfg.acc.range = BMI2_ACC_RANGE_2G;
      config.cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;
      config.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

      /* Set the gyro configurations */
      rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

      if (rslt == BMI2_OK)
      {
        /* Map interrupt to pins */
        rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev);
      }
    }
  }

  return rslt;
}

int8_t bmi2_gyro_set_config(struct bmi2_dev *bmi2_dev)
{
   /* Variable to define result */
    int8_t rslt;

    /* Initialize interrupts for gyroscope */
    uint8_t sens_int = BMI2_DRDY_INT;

    /* List the sensors which are required to enable */
    uint8_t sens_list = BMI2_GYRO;

    /* Structure to define the type of the sensor and its configurations */
    struct bmi2_sens_config config;

    /* Configure type of feature */
    config.type = BMI2_GYRO;

    /* Enable the selected sensors */
    rslt = bmi2_sensor_enable(&sens_list, 1, bmi2_dev);

    if (rslt == BMI2_OK)
    {
        /* Get default configurations for the type of feature selected */
        rslt = bmi2_get_sensor_config(&config, 1, bmi2_dev);

        if (rslt == BMI2_OK)
        {
            /* The user can change the following configuration parameter according to their requirement */
            /* Output data Rate. By default ODR is set as 200Hz for gyro */
            config.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
            /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps */
            config.cfg.gyr.range = BMI2_GYR_RANGE_2000;
            config.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
            config.cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;
            config.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

            /* Set the gyro configurations */
            rslt = bmi2_set_sensor_config(&config, 1, bmi2_dev);

            if (rslt == BMI2_OK)
            {
                /* Map interrupt to pins */
                rslt = bmi2_map_data_int(sens_int, BMI2_INT2, bmi2_dev);
            }
        }
    }

    return rslt;
}

void setup() {


  /* Variable to define result */
  int8_t rslt;
  pinMode(21, OUTPUT);
  pinMode(BMI270_CS, OUTPUT);
  digitalWrite(BMI270_CS, HIGH);
  //  Wire.setModule(0);
  Serial.begin(115200);

  spi_bus = BMI270_CS;
  /* To initialize the hal function */

  SPI.begin();
  bmi2.intf_ptr = &spi_bus;
  bmi2.intf = BMI2_SPI_INTF;
  bmi2.read = BMI270_read_spi;
  bmi2.write = BMI270_write_spi;
  bmi2.read_write_len = 32;
  bmi2.delay_us = bmi2xy_hal_delay_usec;

  /* Config file pointer should be assigned to NULL, so that default file address is assigned in bmi270_init */
  bmi2.config_file_ptr = NULL;

  /* Initialize bmi270 */
  rslt = bmi270_init(&bmi2);

  Serial.println("bmi270_init done");

  rslt = bmi2_accel_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    Serial.println("Accel data Ready : X Y Z ");
  }

  rslt = bmi2_gyro_set_config(&bmi2);

  if (rslt == BMI2_OK)
  {
    Serial.println("Gyro data Ready : X Y Z ");
  }
  
  Timer2.attachInterrupt(RawDataHandler).setFrequency(50).start();
}

void loop() {

}

void RawDataHandler()
{
  float accel_x = 0, accel_y = 0, accel_z = 0;

  sensor_data.type = BMI2_ACCEL_GYRO;
  bmi2_get_sensor_data(&sensor_data, 1, &bmi2);

  Serial.print("Accel x = ");
  Serial.print(sensor_data.sens_data.acc.x);
  Serial.print("\t");
  Serial.print("Accel y = ");
  Serial.print(sensor_data.sens_data.acc.y);
  Serial.print("\t");
  Serial.print("Accel z = ");
  Serial.print(sensor_data.sens_data.acc.z);
  Serial.print("\t");

  Serial.print("Gyro x = ");
  Serial.print(sensor_data.sens_data.gyr.x);
  Serial.print("\t");
  Serial.print("Gyro y = ");
  Serial.print(sensor_data.sens_data.gyr.y);
  Serial.print("\t");
  Serial.print("Gyro z = ");
  Serial.println(sensor_data.sens_data.gyr.z);

}
