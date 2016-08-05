#include "sd.h"

// ----------------------- Extern variables ----------------------- //

//unsigned char SDIO_sdioBuffer[SDIO_BUFFER_SIZE] = {0};
//uint32_t SDIO_totalBytesRead = 0;
//uint32_t SDIO_bytesRead = 0;
//bool SDIO_isFileReadStreamInProgress = false;
char SDIO_currentPath[SDIO_CURRENT_PATH_MAX_LENGTH] = "/";

struct SDIO_fileStruct SDIO_files[SDIO_FILES_TO_VIEW_MAX];
uint16_t SDIO_filesNum = 0;

// ----------------------- Private variables ----------------------- //
//static FIL 		 _file;
static DIR _directory;
static FILINFO _fileInfo;
static FATFS_t _fatFSStatus;
static FRESULT _fresult;
//static char _filePath[SDIO_CURRENT_PATH_MAX_LENGTH] = {0};
#if _USE_LFN
static char _lfName[_MAX_LFN + 1]; /* Buffer to store the LFN */
#endif

BaseType_t SDIO_isSDCardAccessible(void) {
	_fatFSStatus = UB_Fatfs_CheckMedia(MMC_0);
	return (_fatFSStatus == FATFS_OK);
}

BaseType_t SDIO_mountSDCard(void) {
	_fatFSStatus = UB_Fatfs_Mount(MMC_0);
	return (_fatFSStatus == FATFS_OK);
}
#ifdef qwe
bool SDIO_openFileForRead(TCHAR * fileName)
{
	strcpy(_filePath, SDIO_currentPath);
	strcat(_filePath, "/");
	strcat(_filePath, fileName);

	_fatFSStatus = UB_Fatfs_OpenFile(&_file, _filePath, F_RD);
	return (_fatFSStatus == FATFS_OK);
}

void SDIO_gotoFolder(char * folderName)
{
	if(SDIO_currentPath[1] != 0x00) strcat(SDIO_currentPath, "/");
	strcat(SDIO_currentPath, folderName);

	SDIO_scanFiles();
}

void SDIO_gotoParentFolder(void)
{
	static signed int i;

	for(i = (SDIO_CURRENT_PATH_MAX_LENGTH - 1); i >= 0; i--)
	{
		if(SDIO_currentPath[i] == '/')
		{
			if(i > 0) SDIO_currentPath[i] = 0x00;
			else SDIO_currentPath[i+1] = 0x00;
			break;
		}
	}

	SDIO_scanFiles();
}

#endif

BaseType_t SDIO_scanFiles(void) {
	static uint16_t i;

	i = 0;
	SDIO_filesNum = 0;

	_fresult = f_opendir(&_directory, SDIO_currentPath); /* Open the directory */
	if (_fresult == FR_OK) {
		for (; i < SDIO_FILES_TO_VIEW_MAX; i++) {
			_fresult = f_readdir(&_directory, &_fileInfo); /* Read a directory item */
			if (_fresult != FR_OK || _fileInfo.fname[0] == 0x00)
				break; /* Break on error or end of dir */
			if (_fileInfo.fname[0] == '.')
				continue; /* Ignore dot entry */

			_fileInfo.lfname = _lfName;
			_fileInfo.lfsize = sizeof _lfName;

#if _USE_LFN
			if (*_fileInfo.lfname)
				strcpy(SDIO_files[SDIO_filesNum].fileName, _fileInfo.lfname);
			else
				strcpy(SDIO_files[SDIO_filesNum].fileName, _fileInfo.fname);
#else
			strcpy(SDIO_files[SDIO_filesNum].fileName, _fileInfo.fname);
#endif

			SDIO_files[SDIO_filesNum].fileName[(SDIO_FILE_STRUCT_NAME_MAX_LENGTH
					- 1)] = 0x00;
			SDIO_files[SDIO_filesNum].isDirectory =
					(_fileInfo.fattrib & AM_DIR);

			SDIO_filesNum++;
		}
	}

	return _fresult == FR_OK;
}

#ifdef qwe

void SDIO_runFileStream(void)
{
	xTaskCreate(SDIO_manageFileStreamTask,
			TASKS_SDIO_MANAGE_FILE_STREAM_TASK_NAME,
			TASKS_SDIO_MANAGE_FILE_STREAM_STACK_SIZE,
			NULL,
			TASKS_SDIO_MANAGE_FILE_STREAM_PRIORITY,
			&TASKS_SDIO_manageFileStreamTask);
}

void SDIO_resumeFileStream(void)
{
	vTaskResume(TASKS_SDIO_manageFileStreamTask);
}

void SDIO_stopFileStream(void)
{
	if(SDIO_isFileReadStreamInProgress == false) return;
	SDIO_isFileReadStreamInProgress = false;

	UB_Fatfs_CloseFile(&_file);
	vTaskDelete(TASKS_SDIO_manageFileStreamTask);
}

// ----------------------- TASK: Read File Until End ----------------------- //
void SDIO_manageFileStreamTask(void *pvParameters)
{
	SDIO_bytesRead = 0;
	SDIO_totalBytesRead = 0;
	_fresult = FR_OK;
	SDIO_isFileReadStreamInProgress = true;

	while(_fresult == FR_OK)
	{
		if(f_tell(&_file) >= f_size(&_file)) break;

		_fresult = f_read(&_file, SDIO_sdioBuffer, SDIO_BUFFER_SIZE, &SDIO_bytesRead);
		if(_fresult == FR_OK)
		{
			SDIO_totalBytesRead += SDIO_bytesRead;

			// !!! SUSPEND TASK until audio buffer will be empty
			// To resume use:  SDIO_resumeFileStream();
			vTaskSuspend(TASKS_SDIO_manageFileStreamTask);
		}
		else printf("Error read file stream!!!\n\r");
	}

	SDIO_stopFileStream();
}

// ------ "stm32_ub_sdcard.h" callbacks ------ //
/*
 void SDIO_onTaskSuspend(void)
 {

 }
 void SDIO_onTaskResume(void)
 {

 }
 */

#endif

void getFreeSize(uart_t *u) {
	FATFS *fs;
	DWORD fre_clust, fre_sect, tot_sect;
	char buf[100];

	/* Get volume information and free clusters of drive 1 */
	FRESULT res = f_getfree("0:", &fre_clust, &fs);
	if (res == FR_OK) {
		/* Get total sectors and free sectors */
		tot_sect = (fs->n_fatent - 2) * fs->csize;
		fre_sect = fre_clust * fs->csize;

		/* Print the free space (assuming 512 bytes/sector) */
		sprintf(buf, "%10lu KiB total drive space.\n%10lu KiB available.\n",
				tot_sect / 2, fre_sect / 2);
		PutStringUart(u, buf);
	} else {
		sprintf(buf, "res = %d\n",
						res);
				PutStringUart(u, buf);
	}
}
