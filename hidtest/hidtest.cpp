/*******************************************************
 Windows HID simplification

 Alan Ott
 Signal 11 Software

 8/22/2009

 Copyright 2009

 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/

#include <stdio.h>
#include <wchar.h>
#include <string.h>
#include <stdlib.h>
#include "hidapi.h"


// Headers needed for sleeping.
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "SMN812-548_V1013.h"	// New Firmware File

hid_device* handle;

const unsigned char* CurrentHexFile = NULL;

// PIC32 BOOTLOADER COMMANDS
#define READ_BOOT_INFO	0x01
#define ERASE_FLASH		0x02
#define PROGRAM_FLASH	0x03
#define READ_CRC		0x04
#define JMP_TO_APP		0x05

// PIC32 COMMAND CHARACTERS
#define SOH				0x01
#define EOT				0x04
#define DLE				0x10

// TX/RX BUFFERS
unsigned char TxPacket[256];	// RAW TX BUFFER
unsigned short TxPacketLen;

unsigned char TxData[256];		// UNPROCESSED TX DATA
unsigned short TxDataLen;

unsigned char RxPacket[256];	// RAW RX BUFFER
unsigned short RxPacketLen;

unsigned char RxData[256];		// RX DATA PROCESSED
unsigned short RxDataLen;
bool RxFrameValid = FALSE;		// CRC CHECK OUTCOME

// VIRTUAL FLASH
#define KB (1024)
#define MB (KB*KB)

// 5MB
static unsigned char VirtualFlash[5 * MB];
unsigned short crc2;

#define BOOT_SECTOR_BEGIN 0x9FC00000
#define APPLICATION_START 0x9D000000
#define PA_TO_VFA(x)	(x-APPLICATION_START)
#define PA_TO_KVA0(x)   (x|0x80000000)

#define DATA_RECORD 		0
#define END_OF_FILE_RECORD 	1
#define EXT_SEG_ADRS_RECORD 2
#define EXT_LIN_ADRS_RECORD 4

// HEX FILE POINTER
unsigned int HexFilePtr = 0;
char HexRec[1000];


typedef struct
{
	unsigned char RecDataLen;
	unsigned int Address;
	unsigned int MaxAddress;
	unsigned int MinAddress;
	unsigned char RecType;
	unsigned char* Data;
	unsigned char CheckSum;
	unsigned int ExtSegAddress;
	unsigned int ExtLinAddress;
}T_HEX_RECORD;


/*****************************************************************************
 * Static table used for the table_driven CRC implementation
 *****************************************************************************/
static const unsigned short crc_table[16] =
{
	0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
	0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef
};


/*****************************************************************************
 * Update the crc value with new data.
 *
 * \param	data: Pointer to a buffer of \a data_len bytes
 * \param	len: Number of bytes in the \a data buffer
 * \return	The updated crc value
 *****************************************************************************/
unsigned short CalculateCrc(unsigned char* data, unsigned int len)
{
	unsigned int i;
	unsigned short crc = 0;

	while (len--)
	{
		i = (crc >> 12) ^ (*data >> 4);
		crc = crc_table[i & 0x0F] ^ (crc << 4);
		i = (crc >> 12) ^ (*data >> 0);
		crc = crc_table[i & 0x0F] ^ (crc << 4);
		data++;
	}

	return (crc & 0xFFFF);
}


/****************************************************************************
 * Resets hex file pointer
 *****************************************************************************/
void ResetHexFilePointer(void)
{
	HexFilePtr = 0;
}


/****************************************************************************
* Gets next Hex record from the HexFile
*
* \param  HexRec: Pointer to HexRec.
* \param  BuffLen: Buffer Length
* \return Length of the hex record in bytes.
*****************************************************************************/
unsigned short GetNextHexRecord(unsigned short offset)
{
	unsigned short i = 0;
	unsigned short j = 0 + offset;
	char Temp[1000];
	char format[5] = { '0', 'x', NULL, NULL, NULL };

	memset(Temp, 0, sizeof(Temp));

	if (CurrentHexFile[HexFilePtr] != 58)	// Check for start byte
		return 0;						// Not a valid Hex record

	HexFilePtr++;

	while (1)
	{
		Temp[i] = CurrentHexFile[HexFilePtr];	// Copy data bytes
		i++;
		HexFilePtr++;

		if ((CurrentHexFile[HexFilePtr] == 13) && (CurrentHexFile[HexFilePtr + 1] == 10))	// Look at next 2 values for end
			break;		// End of record
	}
	HexFilePtr++;
	HexFilePtr++;	// Pointer should now be at start of new record
	i = 0; // POSSIBLE -1?

	while (1)
	{
		format[2] = Temp[i++];
		format[3] = Temp[i++];
		if ((format[2] == NULL) || (format[3] == NULL))
		{
			// Not a valid ASCII. Stop conversion and break.
			i -= 2;
			return(i / 2);
		}
		else
		{
			// Convert ASCII to hex.
			//sscanf(format, "%x", &HexRec[j]);
			unsigned int temp;
			sscanf_s(format, "%x", &temp);  // Also using sscanf_s for security
			HexRec[j] = (char)temp;

			j++;
		}
	}
	return 0;	// Return length of record
}


/****************************************************************************
 * Verifies flash
 *
 * \param  StartAddress: Pointer to program start address
 * \param  ProgLen:	Pointer to Program length in bytes
 * \param  crc : Pointer to CRC
 *****************************************************************************/
void PrintBlockProgress(int percentage)
{
	const int totalBlocks = 40;
	int filledBlocks = totalBlocks * percentage / 100;

	printf("\rUpdating: ");
	for (int i = 0; i < totalBlocks; i++)
	{
		if (i < filledBlocks)
			printf(">");  // Full block
		else
			printf(" ");  // Light block
	}
	printf(" %3d%%", percentage);
	fflush(stdout);
}

void VerifyFlash(unsigned int* StartAdress, unsigned int* ProgLen, unsigned short* crc)
{
	unsigned short HexRecLen;
	T_HEX_RECORD HexRecordSt;
	unsigned int VirtualFlashAdrs;
	unsigned int ProgAddress;

	// Virtual Flash Erase (Set all bytes to 0xFF)
	memset((void*)VirtualFlash, 0xFF, sizeof(VirtualFlash));

	// Start decoding the hex file and write into virtual flash
	// Reset file pointer.
	HexFilePtr = 0;

	// Reset max address and min address.
	HexRecordSt.MaxAddress = 0;
	HexRecordSt.MinAddress = 0xFFFFFFFF;

	while ((HexRecLen = GetNextHexRecord(0)) != 0)
	{
		HexRecordSt.RecDataLen = HexRec[0];
		HexRecordSt.RecType = HexRec[3];
		HexRecordSt.Data = (unsigned char*)&HexRec[4];

		switch (HexRecordSt.RecType)
		{

		case DATA_RECORD:  //Record Type 00, data record.
			HexRecordSt.Address = (((HexRec[1] << 8) & 0x0000FF00) | (HexRec[2] & 0x000000FF)) & (0x0000FFFF);
			HexRecordSt.Address = HexRecordSt.Address + HexRecordSt.ExtLinAddress + HexRecordSt.ExtSegAddress;

			ProgAddress = PA_TO_KVA0(HexRecordSt.Address);

			if (ProgAddress < BOOT_SECTOR_BEGIN) // Make sure we are not writing boot sector.
			{
				if (HexRecordSt.MaxAddress < (ProgAddress + HexRecordSt.RecDataLen))
				{
					HexRecordSt.MaxAddress = ProgAddress + HexRecordSt.RecDataLen;
				}

				if (HexRecordSt.MinAddress > ProgAddress)
				{
					HexRecordSt.MinAddress = ProgAddress;
				}

				VirtualFlashAdrs = PA_TO_VFA(ProgAddress); // Program address to local virtual flash address

				memcpy((void*)&VirtualFlash[VirtualFlashAdrs], HexRecordSt.Data, HexRecordSt.RecDataLen);
			}
			break;

		case EXT_SEG_ADRS_RECORD:  // Record Type 02, defines 4 to 19 of the data address.
			HexRecordSt.ExtSegAddress = ((HexRecordSt.Data[0] << 16) & 0x00FF0000) | ((HexRecordSt.Data[1] << 8) & 0x0000FF00);
			HexRecordSt.ExtLinAddress = 0;
			break;

		case EXT_LIN_ADRS_RECORD:
			HexRecordSt.ExtLinAddress = ((HexRecordSt.Data[0] << 24) & 0xFF000000) | ((HexRecordSt.Data[1] << 16) & 0x00FF0000);
			HexRecordSt.ExtSegAddress = 0;
			break;


		case END_OF_FILE_RECORD:  //Record Type 01
		default:
			HexRecordSt.ExtSegAddress = 0;
			HexRecordSt.ExtLinAddress = 0;
			break;
		}
	}

	HexRecordSt.MinAddress -= HexRecordSt.MinAddress % 4;
	HexRecordSt.MaxAddress += HexRecordSt.MaxAddress % 4;

	*ProgLen = HexRecordSt.MaxAddress - HexRecordSt.MinAddress;
	*StartAdress = HexRecordSt.MinAddress;
	VirtualFlashAdrs = PA_TO_VFA(HexRecordSt.MinAddress);
	*crc = CalculateCrc((unsigned char*)&VirtualFlash[VirtualFlashAdrs], *ProgLen);
}


/****************************************************************************
 *  Gets locally calculated CRC
 *
 * \param
 * \param
 * \param
 * \return 16 bit CRC
 *****************************************************************************/
unsigned short CalculateFlashCRC()
{
	unsigned int StartAddress, Len;
	unsigned short crc;
	VerifyFlash((unsigned int*)&StartAddress, (unsigned int*)&Len, (unsigned short*)&crc);
	return crc;
}


/*****************************************************************************
 * Creates the TX Frame to send a command
 *
 * \param	cmd: Command to Send
 *****************************************************************************/
bool CreateTxPacket(char cmd)
{
	unsigned short crc;

	unsigned int StartAddress, Len;
	unsigned short TxDataLen = 0;
	unsigned short HexRecLen;
	UINT totalRecords = 0;
	TxPacketLen = 0;
	unsigned short j = 0;

	//printf("Inside CreateTxPacket, cmd = %02X\r\n", cmd);

	switch (cmd)
	{
	case READ_BOOT_INFO:
		TxData[TxDataLen++] = cmd;
		break;

	case ERASE_FLASH:
		TxData[TxDataLen++] = cmd;
		break;

	case JMP_TO_APP:
		TxData[TxDataLen++] = cmd;
		break;

	case PROGRAM_FLASH:
		TxData[TxDataLen++] = cmd;

		HexRecLen = GetNextHexRecord(0);
		if (HexRecLen == 0)
		{
			//Not a valid hex file.
			return false;
		}

		while (1)
		{
			TxData[TxDataLen++] = HexRec[TxDataLen - 1];
			if (TxDataLen == HexRecLen + 1)
				break;
		}
		TxDataLen--;

		while (totalRecords)
		{
			//printf("Inside CreateTxPacket, totalRecords = %d\r\n", totalRecords);

			HexRecLen = GetNextHexRecord(TxDataLen);
			j = HexRecLen + TxDataLen;
			while (1)
			{
				//printf("Inside CreateTxPacket, TxDataLen = %d\r\n", TxDataLen);

				TxData[TxDataLen++] = HexRec[TxDataLen - 1];
				if (TxDataLen == j + 1)
					break;
			}
			TxDataLen--;
			totalRecords--;
		}
		TxDataLen++;

		//printf("Exited both while (totalRecords) and while(1) loops in CreateTxPacket\r\n");

		break;

	case READ_CRC:
		TxData[TxDataLen++] = cmd;
		VerifyFlash((unsigned int*)&StartAddress, (unsigned int*)&Len, (unsigned short*)&crc);
		TxData[TxDataLen++] = (StartAddress);
		TxData[TxDataLen++] = (StartAddress >> 8);
		TxData[TxDataLen++] = (StartAddress >> 16);
		TxData[TxDataLen++] = (StartAddress >> 24);
		TxData[TxDataLen++] = (Len);
		TxData[TxDataLen++] = (Len >> 8);
		TxData[TxDataLen++] = (Len >> 16);
		TxData[TxDataLen++] = (Len >> 24);
		//TxData[TxDataLen++] = (char)crc;
		//TxData[TxDataLen++] = (char)(crc >> 8);
		break;

	default:
		return false;
		break;

	}

	// Calculate CRC for the frame.
	crc = CalculateCrc(TxData, TxDataLen);
	TxData[TxDataLen++] = (char)crc;
	TxData[TxDataLen++] = (char)(crc >> 8);

	// REPORT ID 0
	TxPacket[TxPacketLen++] = 0x00;

	// SOH: Start of header
	TxPacket[TxPacketLen++] = SOH;

	// Form TxPacket. Insert DLE in the data field whereever SOH and EOT are present.
	for (int i = 0; i < TxDataLen; i++)
	{
		////printf("Inside for (int i = 0; i < TxDataLen; i++) in CreateTxPacket, TxDataLen = %d\r\n", TxDataLen);

		if ((TxData[i] == EOT) || (TxData[i] == SOH) || (TxData[i] == DLE))
		{
			TxPacket[TxPacketLen++] = DLE;
		}
		TxPacket[TxPacketLen++] = TxData[i];
	}

	// EOT: End of transmission
	TxPacket[TxPacketLen++] = EOT;

	//printf("Exiting CreateTxPacket, TxPacketLen = %d\r\n", TxPacketLen);

	return true;
}


/****************************************************************************
 *  Checks the Rx Packet
 *
 * \param  buff: Pointer to the data buffer
 * \param  buffLen: Buffer length
 *****************************************************************************/
void DecodeRxPacket()
{
	static bool Escape = false;
	unsigned short crc;
	unsigned short bufferlocation = 0;	//
	RxPacketLen = sizeof(RxPacket);
	RxFrameValid = FALSE;

	while (RxPacketLen > 0)
	{
		RxPacketLen--;

		switch (RxPacket[bufferlocation])
		{
		case SOH: // Start of header
			if (Escape)
			{
				// Received byte is not SOH, but data.
				RxData[RxDataLen++] = RxPacket[bufferlocation];
				// Reset Escape Flag.
				Escape = FALSE;
			}
			else
			{
				// Received byte is indeed a SOH which indicates start of new frame.
				RxDataLen = 0;
			}
			break;

		case EOT: // End of transmission
			if (Escape)
			{
				// Received byte is not EOT, but data.
				RxData[RxDataLen++] = RxPacket[bufferlocation];
				// Reset Escape Flag.
				Escape = FALSE;
			}
			else
			{
				// Received byte is indeed a EOT which indicates end of frame.
				// Calculate CRC to check the validity of the frame.
				if (RxDataLen > 1)
				{
					crc = (RxData[RxDataLen - 2]) & 0x00ff;
					crc = crc | ((RxData[RxDataLen - 1] << 8) & 0xFF00);
					if ((CalculateCrc(RxData, (RxDataLen - 2)) == crc) && (RxDataLen > 2))
					{
						// CRC matches and frame received is valid.
						RxFrameValid = TRUE;
						RxData[RxDataLen - 2] = 0;
						RxData[RxDataLen - 1] = 0;
					}
				}
			}
			break;

		case DLE: // Escape character received.
			if (Escape)
			{
				// Received byte is not ESC but data.
				RxData[RxDataLen++] = RxPacket[bufferlocation];
				// Reset Escape Flag.
				Escape = FALSE;
			}
			else
			{
				// Received byte is an escape character. Set Escape flag to escape next byte.
				Escape = TRUE;
			}
			break;

		default: // Data field.
			RxData[RxDataLen++] = RxPacket[bufferlocation];
			// Reset Escape Flag.
			Escape = FALSE;
			break;

		}
		bufferlocation++;
	}
}

#define LOAD_HEX

int main(int argc, char* argv[])
{
	int res;
	//unsigned char buf[256];
#define MAX_STR 255
	wchar_t wstr[MAX_STR];
	//int i;
	unsigned short LocalCRC;
	unsigned short RemoteCRC;
	unsigned char* hexBuffer = NULL;

#ifdef WIN32
	UNREFERENCED_PARAMETER(argc);
	UNREFERENCED_PARAMETER(argv);
#endif

	//struct hid_device_info *devs, *cur_dev;

#if defined(LOAD_HEX)

	// Check command line args
	if (argc < 2)
	{
		printf("Usage: %s <firmware.hex>\n", argv[0]);
		system("pause");
		return -1;
	}

	// Load hex file
	FILE* hexFile = NULL;
	errno_t err = fopen_s(&hexFile, argv[1], "rb");
	if (err != 0 || !hexFile)
	{
		//printf("Error: Cannot open file %s\n", argv[1]);
		printf("Error: Invalid or corrupted hex file\n");
		system("pause");
		return -1;
	}

	// Seek to end of file
	if (fseek(hexFile, 0, SEEK_END) != 0)
	{
		//printf("Error: fseek failed\n");
		printf("Error: Invalid or corrupted hex file\n");
		fclose(hexFile);
		system("pause");
		return -1;
	}

	// Get file size
	long fileSize = ftell(hexFile);
	if (fileSize < 0)
	{
		//printf("Error: ftell failed\n");
		printf("Error: Invalid or corrupted hex file\n");
		fclose(hexFile);
		system("pause");
		return -1;
	}

	// Seek back to start of file
	if (fseek(hexFile, 0, SEEK_SET) != 0)
	{
		//printf("Error: fseek failed\n");
		printf("Error: Invalid or corrupted hex file\n");
		fclose(hexFile);
		system("pause");
		return -1;
	}

	// Sanity limit: 10 MB max
	const long maxFileSize = 10 * 1024 * 1024;
	if (fileSize == 0) // Empty file
	{
		//printf("Error: Empty hex file\n");
		printf("Error: Invalid or corrupted hex file\n");
		fclose(hexFile);
		system("pause");
		return -1;
	}
	if (fileSize > maxFileSize) // Too large
	{
		//printf("Error: Hex file too large (%ld bytes)\n", fileSize);
		printf("Error: Invalid or corrupted hex file\n");
		fclose(hexFile);
		system("pause");
		return -1;
	}

	// Allocate buffer to hold the file
	size_t fileSizeU = (size_t)fileSize;
	hexBuffer = (unsigned char*)malloc(fileSizeU + 1);
	if (hexBuffer == NULL) // Malloc failed
	{
		//printf("Error: malloc failed\n");
		fclose(hexFile);
		system("pause");
		return -1;
	}

	// Read the file into the buffer
	size_t readCount = fread(hexBuffer, 1, fileSizeU, hexFile);
	if (readCount != fileSizeU) // fread failed
	{
		//printf("Error: fread failed (%zu of %zu)\n", readCount, fileSizeU);
		free(hexBuffer);
		fclose(hexFile);
		system("pause");
		return -1;
	}

	hexBuffer[readCount] = 0;
	fclose(hexFile); // Close the file

	// Check for valid hex file (must start with ':')
	if (hexBuffer[0] != ':')
	{
		//printf("Error: Invalid hex file (first char 0x%02X)\n", (unsigned char)hexBuffer[0]);
		free(hexBuffer);

		system("pause");
		return -1;
	}

	//printf("Loaded firmware: %s (%ld bytes)\n\n", argv[1], (long)fileSizeU);

	// Set global pointer for all functions to use
	CurrentHexFile = hexBuffer;
	ResetHexFilePointer();  // Reset to start

	// Print the first 100 bytes of the hex file
	/*printf("First 1000 bytes of hex file:\n");
	for (size_t i = 0; i < 1000 && i < fileSizeU; i++)
	{
		// If its a new line, print a new line inform user
		if (hexBuffer[i] == 13 && hexBuffer[i + 1] == 10)
			printf("\nnew line ");
		printf("%c", hexBuffer[i]);
	}*/

#endif // LOAD_HEX

	if (hid_init())
	{
		//printf("hid_init failed\n");
		return -1;
	}

	// Open the device using the VID, PID,
	// and optionally the Serial number.
	// handle = hid_open(0x4d8, 0x3f, L"12345");

	/*printf("Searching for Device...\n\n");

	while(1)
	{
		handle = hid_open(0x4d8, 0x003c, NULL);
		if (!handle) {
			#ifdef WIN32
				Sleep(500);
			#else
				usleep(500 * 1000);
			#endif
		}
		else
			break;
	}*/

	int timeout_seconds = 60;
	int attempts = 0;
	int max_attempts = timeout_seconds * 2;
	bool init = true;

	while (attempts < max_attempts)
	{
		handle = hid_open(0x4d8, 0x003c, NULL);
		if (handle)
		{
			printf("\n");
			break;
		}

		attempts++;

		// Update countdown every 2 attempts (every second)
		if ((attempts % 2 == 0) || init)
		{
			init = false;
			int remaining = timeout_seconds - (attempts / 2);
			printf("\rSearching for Device... %2d seconds remaining", remaining);
			fflush(stdout);  // Force output immediately
		}

#ifdef WIN32
		Sleep(500);
#else
		usleep(500 * 1000);
#endif
	}

	if (!handle)
	{
		printf("\rSearching for Device...  0 seconds remaining");
		printf("\nDevice not found. Exiting.\n");
		hid_exit();
#ifdef WIN32
		system("pause");
#endif
		return -1;
	}

	printf("Device Detected:\n\n");
	// Read the Manufacturer String
	wstr[0] = 0x0000;
	res = hid_get_manufacturer_string(handle, wstr, MAX_STR);
	if (res < 0)
		printf("Unable to read manufacturer ID\n");
	else
		printf("Manufacturer ID: %ls\n", wstr);
	// Read the Product String
	wstr[0] = 0x0000;
	res = hid_get_product_string(handle, wstr, MAX_STR);
	if (res < 0)
		printf("Unable to read product ID\n");
	else
		printf("Product ID: %ls\n", wstr);
	// Read the Serial Number String (UID)
	wstr[0] = 0x0000;
	res = hid_get_serial_number_string(handle, wstr, MAX_STR);
	if (res < 0)
		printf("Unable to read Device Type\n");
	else
		printf("Device Type: %ls\n", wstr);

	//printf("calling hid_set_nonblocking(handle, 0)\n");
	hid_set_nonblocking(handle, 0);		// Set the hid_read() function to be non-blocking.
	//printf("hid_set_nonblocking() complete\n\n");

	int retryCount = 0;
	const int MAX_RETRIES = 5;
	// REQUEST BOOTLOADER VERSION INFO
	while (!RxFrameValid && retryCount < MAX_RETRIES)
	{
		//printf("Attempt %d: Calling CreateTxPacket(READ_BOOT_INFO)\n", retryCount + 1);
		CreateTxPacket(READ_BOOT_INFO);

		int writeResult = hid_write(handle, TxPacket, TxPacketLen);
		//printf("hid_write() returned %d\n", writeResult);

		if (writeResult < 0)
		{
			const wchar_t* err = hid_error(handle);
			//printf("Write error: %ls\n", err ? err : L"Unknown");

			// CRITICAL: Close and reopen to clear stuck overlapped I/O
			//printf("Resetting device connection...\n");
			hid_close(handle);

#ifdef WIN32
			Sleep(1000);
#else
			usleep(1000 * 1000);
#endif

			handle = hid_open(0x4d8, 0x003c, NULL);
			if (!handle)
			{
				//printf("Failed to reopen device!\n");
				break;
			}

			hid_set_nonblocking(handle, 0);  // Keep blocking mode

#ifdef WIN32
			Sleep(1000);
#else
			usleep(1000 * 1000);
#endif

			retryCount++;
			continue;
		}

		// Read with timeout (10 seconds for first attempt)
		int timeout_ms = (retryCount == 0) ? 10000 : 5000;
		res = hid_read_timeout(handle, RxPacket, sizeof(RxPacket), timeout_ms);

		if (res > 0)
		{
			//printf("Received %d bytes\n", res);
			DecodeRxPacket();

			if (RxFrameValid)
			{
				unsigned short boot_version = (RxData[1] * 10) + RxData[2];
				printf("Bootloader Version: %u\n\n", boot_version);
				break;  // Success!
			}
			else
			{
				//printf("CRC Error - retrying...\n");
			}
		}
		else if (res == 0)
		{
			printf("Timeout - no response received\n");

			// Close and reopen after timeout too
			//printf("Resetting device connection after timeout...\n");
			hid_close(handle);

#ifdef WIN32
			Sleep(1000);
#else
			usleep(1000 * 1000);
#endif

			handle = hid_open(0x4d8, 0x003c, NULL);
			if (!handle)
			{
				//printf("Failed to reopen device!\n");
				break;
			}

			hid_set_nonblocking(handle, 0);

#ifdef WIN32
			Sleep(1000);
#else
			usleep(1000 * 1000);
#endif
		}
		else
		{
			const wchar_t* err = hid_error(handle);
			//printf("Read error: %ls\n", err ? err : L"Unknown");
		}

		retryCount++;

#ifdef WIN32
		Sleep(500);
#else
		usleep(500 * 1000);
#endif
	}


	if (!RxFrameValid)
	{
		printf("\nFailed to communicate with bootloader after %d attempts\n", MAX_RETRIES);
		printf("Please reset the device and try again.\n");

		if (handle)
			hid_close(handle);

		hid_exit();
		free(hexBuffer);
#ifdef WIN32
		system("pause");
#endif
		return -1;
	}

	// CHECK DEVICE CURRENT VERSION CRC
	printf("Checking Current Firmware Version...\n\n");
	CreateTxPacket(READ_CRC);					// Creates Read CRC Packet
	hid_write(handle, TxPacket, TxPacketLen);	// Send Packet
	res = 0;
	while (res == 0)											// Wait for data
	{
		res = hid_read(handle, RxPacket, sizeof(RxPacket));		// Check Buffer
#ifdef WIN32
		Sleep(500);
#else
		usleep(500 * 1000);
#endif
	}
	DecodeRxPacket();
	if (!RxFrameValid)
	{
		//printf("CRC Error\n");
		while (1);
	}

	RemoteCRC = ((RxData[2] << 8) & 0xFF00) | (RxData[1] & 0x00FF);

	LocalCRC = CalculateFlashCRC();

	// ERASE > PROGRAM > VERIFY
	if (LocalCRC != RemoteCRC)
	{
		printf("Updating Firmware...\n\n");
		printf("DO NOT DISCONNECT THE DEVICE!\n\n");

		// ERASE FLASH
		CreateTxPacket(ERASE_FLASH);
		hid_write(handle, TxPacket, TxPacketLen);
		res = 0;
		while (res == 0)											// Wait for data
		{
			res = hid_read(handle, RxPacket, sizeof(RxPacket));		// Check Buffer
#ifdef WIN32
			Sleep(500);
#else
			usleep(500 * 1000);
#endif
		}

		// PROGRAM FLASH
		//printf("> 0%%\n");
		float UpdateSize = (float)HexFilePtr;
		float UpdateProgress = 0;
		float ProgressBar = 0;
		ResetHexFilePointer();
		bool Programming = 1;
		CreateTxPacket(PROGRAM_FLASH);
		hid_write(handle, TxPacket, TxPacketLen);

		res = 0;
		while (res == 0)											// Wait for data
		{
			res = hid_read(handle, RxPacket, sizeof(RxPacket));		// Check Buffer
#ifdef WIN32
			Sleep(500);
#else
			usleep(500 * 1000);
#endif
		}
		DecodeRxPacket();
		if (!RxFrameValid)
		{
			//printf("CRC Error\n");
			while (1);
		}

		Programming = CreateTxPacket(PROGRAM_FLASH);

		while (Programming)
		{
			hid_write(handle, TxPacket, TxPacketLen);
			res = 0;
			while (res == 0)											// Wait for data
			{
				res = hid_read(handle, RxPacket, sizeof(RxPacket));		// Check Buffer
#ifdef WIN32
	//Sleep(1);
#else
				usleep(500 * 1000);
#endif
			}
			DecodeRxPacket();
			if (!RxFrameValid)
			{
				//printf("CRC Error\n");
				while (1);
			}



			Programming = CreateTxPacket(PROGRAM_FLASH);
			UpdateProgress = ((HexFilePtr / UpdateSize) * 100);
			if (UpdateProgress > (ProgressBar + 2))
			{
				ProgressBar = UpdateProgress;
				PrintBlockProgress((int)ProgressBar);
			}
		}
		printf("\n");
		printf("> 100%%\n");
		printf("Update Complete!\n");

		CreateTxPacket(JMP_TO_APP);
		hid_write(handle, TxPacket, TxPacketLen);
	}
	else
		printf("Loaded firmware version already installed\n");

	hid_close(handle);

	free(hexBuffer);

	/* Free static HIDAPI objects. */
	hid_exit();

#ifdef WIN32
	system("pause");
#endif

	return 0;
}
