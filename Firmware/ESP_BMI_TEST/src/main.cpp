#include <Arduino.h>
#include <SPI.h>
#include "SD_MMC.h"
#include "FS.h"

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

#define BMI270_CS       5

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

//SPISettings spi_set(10E6, MSBFIRST, SPI_MODE0);

void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s\n", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

void setup(void)
{
  Serial.begin(115200);

  delay(2000);
  
//   if(!SD_MMC.begin()){
//       Serial.println("Card Mount Failed");
//       return;
//   }
//   uint8_t cardType = SD_MMC.cardType();

//   if(cardType == CARD_NONE){
//       Serial.println("No SD_MMC card attached");
//       return;
//   }

//   Serial.print("SD_MMC Card Type: ");
//   if(cardType == CARD_MMC){
//       Serial.println("MMC");
//   } else if(cardType == CARD_SD){
//       Serial.println("SDSC");
//   } else if(cardType == CARD_SDHC){
//       Serial.println("SDHC");
//   } else {
//       Serial.println("UNKNOWN");
//   }

//   uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
//   Serial.printf("SD_MMC Card Size: %lluMB\n", cardSize);

//   listDir(SD_MMC, "/", 0);
//   createDir(SD_MMC, "/mydir");
//   listDir(SD_MMC, "/", 0);
//   removeDir(SD_MMC, "/mydir");
//   listDir(SD_MMC, "/", 2);
//   writeFile(SD_MMC, "/hello.txt", "Hello ");
//   appendFile(SD_MMC, "/hello.txt", "World!\n");
//   readFile(SD_MMC, "/hello.txt");
//   deleteFile(SD_MMC, "/foo.txt");
//   renameFile(SD_MMC, "/hello.txt", "/foo.txt");
//   readFile(SD_MMC, "/foo.txt");
//   testFileIO(SD_MMC, "/test.txt");
//   Serial.printf("Total space: %lluMB\n", SD_MMC.totalBytes() / (1024 * 1024));
//   Serial.printf("Used space: %lluMB\n", SD_MMC.usedBytes() / (1024 * 1024));

//   SPI.begin(18,19,23,5);
//   SPI.setClockDivider(SPI_CLOCK_DIV8);
//   //SPI.setDataMode(SPI_MODE3);
//   delay(100);
//   digitalWrite(SS,LOW);

  int8_t rslt;
  pinMode(BMI270_CS, OUTPUT);
  digitalWrite(BMI270_CS, HIGH);
  //  Wire.setModule(0);
  //Serial.begin(115200);

  spi_bus = BMI270_CS;
  /* To initialize the hal function */

  SPI.begin(18,19,23);
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

void loop(void) 
{
   uint8_t chip_id;
//   SPI.beginTransaction(spi_set);
//   digitalWrite(SS,LOW);
//   SPI.transfer(0x80 | 0x00);
//   //SPI.transfer(0x80);
//   SPI.transfer(&chip_id,1);
//   digitalWrite(SS,HIGH);
//   SPI.endTransaction();
//   Serial.println(chip_id);
//   delay(1000);

    BMI270_read_spi(0x00,&chip_id,1,&spi_bus);
    Serial.println(chip_id);
    // RawDataHandler();
    delay(1000);
}
