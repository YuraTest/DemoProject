
#ifndef _SDIO_TASK_H
#define _SDIO_TASK_H

// ----------------------- Include ----------------------- //
#include "FreeRTOS.h"
#include "stm32_ub_fatfs.h"
#include "ff.h"

//#include "TaskHandlers.h"
#include <stdbool.h>
#include <string.h>

// ----------------------- Define ----------------------- //
//#define SDIO_BUFFER_SIZE 									(AUDIO_BUFFER_HALF_LENGTH*2)
#define SDIO_FILE_STRUCT_NAME_MAX_LENGTH 	_MAX_LFN
#define SDIO_CURRENT_PATH_MAX_LENGTH 	128

#define SDIO_FILES_TO_VIEW_MAX 	16


// ----------------------- Struct ----------------------- //
struct SDIO_fileStruct
{
	TCHAR fileName[SDIO_FILE_STRUCT_NAME_MAX_LENGTH];
	BaseType_t isDirectory;
};

// ----------------------- Extern Variables ----------------------- //
/*
extern unsigned char SDIO_sdioBuffer[];
extern uint32_t SDIO_totalBytesRead;
extern uint32_t SDIO_bytesRead;
extern bool SDIO_isFileReadStreamInProgress;
extern struct SDIO_fileStruct SDIO_files[SDIO_FILES_TO_VIEW_MAX];
extern char SDIO_currentPath[SDIO_CURRENT_PATH_MAX_LENGTH];
extern uint16_t SDIO_filesNum;
*/
// ----------------------- Extern Functions ----------------------- //
//void SDIO_manageFileStreamTask(void *pvParameters);
BaseType_t SDIO_mountSDCard(void);			/*	*/
BaseType_t SDIO_isSDCardAccessible(void);	/*  */
//bool SDIO_openFileForRead(char * filepath);
//void SDIO_resumeFileStream(void);
//void SDIO_runFileStream(void);
//void SDIO_stopFileStream(void);
BaseType_t SDIO_scanFiles (void);
//void SDIO_gotoFolder(char * folderName);
//void SDIO_gotoParentFolder(void);

#endif
