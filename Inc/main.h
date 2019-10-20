/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define FINGERPRINT_OK 													0x00
#define FINGERPRINT_PACKETRECIEVEERR 						0x01
#define FINGERPRINT_NOFINGER 										0x02
#define FINGERPRINT_IMAGEFAIL 									0x03
#define FINGERPRINT_IMAGEMESS 									0x06
#define FINGERPRINT_FEATUREFAIL 								0x07
#define FINGERPRINT_NOMATCH 										0x08
#define FINGERPRINT_NOTFOUND 										0x09
#define FINGERPRINT_ENROLLMISMATCH 							0x0A
#define FINGERPRINT_BADLOCATION 								0x0B
#define FINGERPRINT_DBRANGEFAIL 								0x0C
#define FINGERPRINT_UPLOADFEATUREFAIL 					0x0D
#define FINGERPRINT_PACKETRESPONSEFAIL 					0x0E
#define FINGERPRINT_UPLOADFAIL 									0x0F
#define FINGERPRINT_DELETEFAIL 									0x10
#define FINGERPRINT_DBCLEARFAIL 								0x11
#define FINGERPRINT_PASSFAIL 										0x13
#define FINGERPRINT_INVALIDIMAGE 								0x15
#define FINGERPRINT_FLASHERR 										0x18
#define FINGERPRINT_INVALIDREG 									0x1A
#define FINGERPRINT_ADDRCODE 										0x20
#define FINGERPRINT_PASSVERIFY 									0x21

#define FINGERPRINT_STARTCODE_BYTE0							0xEF
#define FINGERPRINT_STARTCODE_BYTE1							0x01

#define FINGERPRINT_COMMANDPACKET 							0x1
#define FINGERPRINT_DATAPACKET 									0x2
#define FINGERPRINT_ACKPACKET 									0x7
#define FINGERPRINT_ENDDATAPACKET 							0x8

#define FINGERPRINT_TIMEOUT 										0xFF
#define FINGERPRINT_BADPACKET 									0xFE

#define FINGERPRINT_GETIMAGE 										0x01
#define FINGERPRINT_IMAGE2TZ 										0x02
#define FINGERPRINT_SEARCH	 										0x04
#define FINGERPRINT_REGMODEL 										0x05
#define FINGERPRINT_STORE 											0x06
#define FINGERPRINT_LOAD 												0x07
#define FINGERPRINT_UPLOAD 											0x08
#define FINGERPRINT_DELETE 											0x0C
#define FINGERPRINT_EMPTY 											0x0D
#define FINGERPRINT_SETPASSWORD 								0x12
#define FINGERPRINT_VERIFYPASSWORD 							0x13
#define FINGERPRINT_HISPEEDSEARCH 							0x1B
#define FINGERPRINT_TEMPLATECOUNT 							0x1D
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
