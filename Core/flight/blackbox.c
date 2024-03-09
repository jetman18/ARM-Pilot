#include "plane.h"
#include "fatfs_sd.h"
#include "fatfs.h"

void blackboxInit(){
FATFS fs;
FIL fil;
/* USER CODE END PD */

  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
	f_mount(&fs, "SD:", 1);
  f_open(&fil, "write.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);
  f_lseek(&fil, fil.fsize);
  f_puts("Hello from Nizar\n", &fil);
  f_close(&fil);
  /* USER CODE END 2 */
}