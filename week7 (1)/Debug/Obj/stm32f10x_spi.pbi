      [�        �      1	�    1	102	�    2	205	�    5	506	�    6	609	�    9	90<	�    <	<0=	�    =	=0@	�	   	 @	@0A	�
   	
 A	A0D	�   
 D	D0E	�    E	E0H	�    H	H0I	�    I	I0L	�    L	L6M	�    M	M6N	�    N	N6O	�    O	O6��D	
 ��ww w���H	 ��#w"
w" ww&��E	 ��&�� ���� ����L	 ���
� �� �
� �� ��M	 ���3
�3 �"�A�3
�3 �"�A��I	 ��'��1	 ���� ����2	 ���&
�& ��4��5	 ��&��  ���%�!& ����6	 ��(�&
�& ��4�
�" ����	@		 ���!�" ���1
#�1#$ �!�9�'�$( ����
A		
 �� �
� ���$
�$% ��(��9	 ���1
#�1 $ �!�9�2
)�2&$ �*�<�7�68 ����<	 ���N
#�N'$ �>�V�&
�&7 ��*�<�;= ����=	 ��!�*�(+ ���5
9�58$ �,�A�$
�$< ��(�
>�
>? ���"
�") ��&�:�9; ���:
#�:=$ �*�B�
@�
?A ���1
,�1*$ �(�?�#
�#: ��'�B�@C ���Q
#�Q+$ �A�Y�/
�/A �"�3�-�,. ���>
D�>B$ �5�K�$
�$- ��(�E�CF ���3
/�3.$ �*�7�G�DH ���
0�
/1 ���%
�%E ��)�2�03 ���4
I�4F$ �+�@�1
�11 �$�5�
J�
GK ���@
4�@2$ �7�S�L�HM ���5�36 ���-
�-I � �1�#
�#4 ��'�;
)�;J$ �3�E�9
#�95$ �)�A   N  ,:K^n|������������������������������������������������������	�	�	�	�	�	�
�
�
�
�������stm32f10x_spi.h stm32f10x_rcc.h CR1_SPE_Set CR1_SPE_Reset I2SCFGR_I2SE_Set I2SCFGR_I2SE_Reset CR1_CRCNext_Set CR1_CRCEN_Set CR1_CRCEN_Reset CR2_SSOE_Set CR2_SSOE_Reset CR1_CLEAR_Mask I2SCFGR_CLEAR_Mask SPI_Mode_Select I2S_Mode_Select I2S2_CLOCK_SRC I2S3_CLOCK_SRC I2S_MUL_MASK I2S_DIV_MASK SPI_I2S_DeInit void SPI_I2S_DeInit(int *) SPIx int * SPI_Init void SPI_Init(int *, int *) SPI_InitStruct I2S_Init void I2S_Init(int *, int *) I2S_InitStruct SPI_StructInit void SPI_StructInit(int *) I2S_StructInit void I2S_StructInit(int *) SPI_Cmd void SPI_Cmd(int *, int) NewState int I2S_Cmd void I2S_Cmd(int *, int) SPI_I2S_ITConfig void SPI_I2S_ITConfig(int *, int, int) SPI_I2S_IT SPI_I2S_DMACmd void SPI_I2S_DMACmd(int *, int, int) SPI_I2S_DMAReq SPI_I2S_SendData void SPI_I2S_SendData(int *, int) Data SPI_I2S_ReceiveData int SPI_I2S_ReceiveData(int *) SPI_NSSInternalSoftwareConfig void SPI_NSSInternalSoftwareConfig(int *, int) SPI_NSSInternalSoft SPI_SSOutputCmd void SPI_SSOutputCmd(int *, int) SPI_DataSizeConfig void SPI_DataSizeConfig(int *, int) SPI_DataSize SPI_TransmitCRC void SPI_TransmitCRC(int *) SPI_CalculateCRC void SPI_CalculateCRC(int *, int) SPI_GetCRC int SPI_GetCRC(int *, int) SPI_GetCRCPolynomial int SPI_GetCRCPolynomial(int *) SPI_BiDirectionalLineConfig void SPI_BiDirectionalLineConfig(int *, int) SPI_Direction SPI_I2S_GetFlagStatus int SPI_I2S_GetFlagStatus(int *, int) SPI_I2S_ClearFlag void SPI_I2S_ClearFlag(int *, int) SPI_I2S_FLAG SPI_I2S_GetITStatus int SPI_I2S_GetITStatus(int *, int) SPI_I2S_ClearITPendingBit void SPI_I2S_ClearITPendingBit(int *, int)    K *U���������������������������	�	�	�	�
�
�
�
�������������������������������������� c:stm32f10x_spi.c@1441@macro@CR1_SPE_Set c:stm32f10x_spi.c@1490@macro@CR1_SPE_Reset c:stm32f10x_spi.c@1562@macro@I2SCFGR_I2SE_Set c:stm32f10x_spi.c@1611@macro@I2SCFGR_I2SE_Reset c:stm32f10x_spi.c@1686@macro@CR1_CRCNext_Set c:stm32f10x_spi.c@1759@macro@CR1_CRCEN_Set c:stm32f10x_spi.c@1808@macro@CR1_CRCEN_Reset c:stm32f10x_spi.c@1880@macro@CR2_SSOE_Set c:stm32f10x_spi.c@1929@macro@CR2_SSOE_Reset c:stm32f10x_spi.c@2007@macro@CR1_CLEAR_Mask c:stm32f10x_spi.c@2056@macro@I2SCFGR_CLEAR_Mask c:stm32f10x_spi.c@2146@macro@SPI_Mode_Select c:stm32f10x_spi.c@2195@macro@I2S_Mode_Select c:stm32f10x_spi.c@2287@macro@I2S2_CLOCK_SRC c:stm32f10x_spi.c@2342@macro@I2S3_CLOCK_SRC c:stm32f10x_spi.c@2397@macro@I2S_MUL_MASK c:stm32f10x_spi.c@2452@macro@I2S_DIV_MASK c:@F@SPI_I2S_DeInit c:stm32f10x_spi.c@3051@F@SPI_I2S_DeInit@SPIx c:@F@SPI_Init c:stm32f10x_spi.c@4260@F@SPI_Init@SPIx c:stm32f10x_spi.c@4279@F@SPI_Init@SPI_InitStruct c:@F@I2S_Init c:stm32f10x_spi.c@7291@F@I2S_Init@SPIx c:stm32f10x_spi.c@7310@F@I2S_Init@I2S_InitStruct c:@F@SPI_StructInit c:stm32f10x_spi.c@12479@F@SPI_StructInit@SPI_InitStruct c:@F@I2S_StructInit c:stm32f10x_spi.c@13672@F@I2S_StructInit@I2S_InitStruct c:@F@SPI_Cmd c:stm32f10x_spi.c@14686@F@SPI_Cmd@SPIx c:stm32f10x_spi.c@14705@F@SPI_Cmd@NewState c:@F@I2S_Cmd c:stm32f10x_spi.c@15374@F@I2S_Cmd@SPIx c:stm32f10x_spi.c@15393@F@I2S_Cmd@NewState c:@F@SPI_I2S_ITConfig c:stm32f10x_spi.c@16468@F@SPI_I2S_ITConfig@SPIx c:stm32f10x_spi.c@16487@F@SPI_I2S_ITConfig@SPI_I2S_IT c:stm32f10x_spi.c@16507@F@SPI_I2S_ITConfig@NewState c:@F@SPI_I2S_DMACmd c:stm32f10x_spi.c@17741@F@SPI_I2S_DMACmd@SPIx c:stm32f10x_spi.c@17760@F@SPI_I2S_DMACmd@SPI_I2S_DMAReq c:stm32f10x_spi.c@17785@F@SPI_I2S_DMACmd@NewState c:@F@SPI_I2S_SendData c:stm32f10x_spi.c@18491@F@SPI_I2S_SendData@SPIx c:stm32f10x_spi.c@18510@F@SPI_I2S_SendData@Data c:@F@SPI_I2S_ReceiveData c:@F@SPI_NSSInternalSoftwareConfig c:stm32f10x_spi.c@19593@F@SPI_NSSInternalSoftwareConfig@SPIx c:stm32f10x_spi.c@19612@F@SPI_NSSInternalSoftwareConfig@SPI_NSSInternalSoft c:@F@SPI_SSOutputCmd c:stm32f10x_spi.c@20352@F@SPI_SSOutputCmd@SPIx c:stm32f10x_spi.c@20371@F@SPI_SSOutputCmd@NewState c:@F@SPI_DataSizeConfig c:stm32f10x_spi.c@21168@F@SPI_DataSizeConfig@SPIx c:stm32f10x_spi.c@21187@F@SPI_DataSizeConfig@SPI_DataSize c:@F@SPI_TransmitCRC c:stm32f10x_spi.c@21638@F@SPI_TransmitCRC@SPIx c:@F@SPI_CalculateCRC c:stm32f10x_spi.c@22154@F@SPI_CalculateCRC@SPIx c:stm32f10x_spi.c@22173@F@SPI_CalculateCRC@NewState c:@F@SPI_GetCRC c:@F@SPI_GetCRCPolynomial c:@F@SPI_BiDirectionalLineConfig c:stm32f10x_spi.c@24369@F@SPI_BiDirectionalLineConfig@SPIx c:stm32f10x_spi.c@24388@F@SPI_BiDirectionalLineConfig@SPI_Direction c:@F@SPI_I2S_GetFlagStatus c:@F@SPI_I2S_ClearFlag c:stm32f10x_spi.c@26937@F@SPI_I2S_ClearFlag@SPIx c:stm32f10x_spi.c@26956@F@SPI_I2S_ClearFlag@SPI_I2S_FLAG c:@F@SPI_I2S_GetITStatus c:@F@SPI_I2S_ClearITPendingBit c:stm32f10x_spi.c@29801@F@SPI_I2S_ClearITPendingBit@SPIx c:stm32f10x_spi.c@29820@F@SPI_I2S_ClearITPendingBit@SPI_I2S_IT     y<invalid loc> C:\Users\pnu2\Downloads\실험_제공_파일\Libraries\STM32F10x_StdPeriph_Driver_v3.5\src\stm32f10x_spi.c 