#include "fatfs.h"
#include "string.h"
#include "timer.h"
#include "utils.h"
#include "blackbox.h"
#define BUFFER_SIZE 400

static FATFS *pfs;
static DWORD fre_clust;
static FATFS _fs;
static uint8_t isSdcard_valid;
static void reverse( char *str, int len);
static int intToStr(black_box_file_t *fs,int x,  char *str, int d);


/*
 * init black box, use for loging data
 *
 */
int black_box_init(){
    isSdcard_valid = 1;
    uint8_t mount = f_mount(&_fs,"", 1);
	if( mount != 0 ){
        isSdcard_valid = 0;
        return -1;
    }

    return 0;
}

int black_box_create_file(black_box_file_t *fs,char *file_name){
   if(isSdcard_valid){
        fs->buffer_index = 0;

        uint8_t open = f_open(&fs->file,file_name, FA_CREATE_ALWAYS | FA_WRITE);
        f_lseek(&fs->file,fs->file.fsize);
        if(open == FR_OK){
            return 0;
        }
   }
   return -1;
}

/*
 *  Convert a number to string and write to buffer
 *  Input (int type [-2147483647 2147483647] )
 */
void black_box_pack_int(black_box_file_t *fs,int val){
    if(isSdcard_valid){
        fs->indexx=0;
        fs->sig = 0;
        // max length of integer is 10 digits and subtract sign -> 11
        char str_[11];
        int len_str = sizeof(str_)/sizeof(char);
        memset(str_,0,len_str);
        if(val < 0){
            val *= -1;
            str_[0] = '-';
            fs->indexx++;
            fs->sig = 1;
        }
        else if(val == 0){
            str_[0]= '0';
        }
        int len = intToStr(fs,val,str_,0);
        // copy str to buffer
        int str_idx = 0;
        int max_index;
        int index_flag;
        if((fs->buffer_index + len) <=  BUFFER_SIZE){
            max_index = fs->buffer_index + len;
            index_flag = 1;
        }
        else{
            max_index = BUFFER_SIZE;
            index_flag = 0;
        }
        for(int j = fs->buffer_index; j < max_index; j++ ){
                fs->buffer[j] = str_[str_idx ++];
        }
        if(index_flag){
            fs->buffer_index += len;
        }
        else{
            fs->buffer_index = BUFFER_SIZE;
        }
    }
}

/*
 * Write str to buffer
 */
void black_box_pack_str(black_box_file_t *fs,const char *c){
    if(isSdcard_valid){
        int i = 0;
        while (c[i]){
            if((fs->buffer_index + i) > BUFFER_SIZE){
                    fs->buffer_index = BUFFER_SIZE;
                    return;
            }
            fs->buffer[fs->buffer_index + i] = c[i];
            i ++;
        }
        fs->buffer_index += i;
    }
}

/*
 * Write buffer to sd card
 */
void black_box_load(black_box_file_t *fs)
 {
      if(isSdcard_valid){
         f_puts(fs->buffer,&fs->file);
      }
 }


/*
 * sync file
 */
void black_box_sync(black_box_file_t *fs)
 { 
     if(isSdcard_valid){
         f_sync(&fs->file);
     }
 }

/*
 * close file
 */
void black_box_close(black_box_file_t *fs)
 {    
     if(isSdcard_valid){
          f_close(&fs->file);
     }
 }

/*
 * get buffer length
 */
 uint16_t black_box_get_buffer_lenght(const black_box_file_t *fs)
 {
     return fs->buffer_index;
 }
 
/*
 * get total space
 */
uint32_t black_box_get_total_space()
 {
     f_getfree("", &fre_clust, &pfs);
     return (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
 }
 
 /*
  * get free space
  */
uint32_t black_box_get_free_space()
 {
     f_getfree("", &fre_clust, &pfs);
     return (uint32_t)(fre_clust * pfs->csize * 0.5);
 }


static void reverse( char *str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}

 static int intToStr(black_box_file_t *fs,int x,  char *str, int d)
{
    while (x) {
        str[fs->indexx++] = (x % 10) + '0';
        x = x / 10;
    }

    while (fs->indexx < d)
        str[fs->indexx++] = '0';
    reverse(str,fs->indexx);
    return fs->indexx;
}
