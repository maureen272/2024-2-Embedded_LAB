#include "stm32f10x.h"

void SysInit(void) {
    printf("SysInit start");
    /* Set HSION bit */
    /* Internal Clock Enable */
    RCC->CR |= (uint32_t)0x00000001; //HSION

    /* Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits */
    RCC->CFGR &= (uint32_t)0xF0FF0000;

    /* Reset HSEON, CSSON and PLLON bits */
    RCC->CR &= (uint32_t)0xFEF6FFFF;

    /* Reset HSEBYP bit */
    RCC->CR &= (uint32_t)0xFFFBFFFF;

    /* Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits */
    RCC->CFGR &= (uint32_t)0xFF80FFFF;

    /* Reset PLL2ON and PLL3ON bits */
    RCC->CR &= (uint32_t)0xEBFFFFFF;

    /* Disable all interrupts and clear pending bits  */
    RCC->CIR = 0x00FF0000;

    /* Reset CFGR2 register */
    RCC->CFGR2 = 0x00000000;
    printf("SysInit end");
}

void SetSysClock(void) {
    printf("SetSysClock start");
    volatile uint32_t StartUpCounter = 0, HSEStatus = 0;
    /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
    /* Enable HSE */
    RCC->CR |= ((uint32_t)RCC_CR_HSEON);
    /* Wait till HSE is ready and if Time out is reached exit */
    do {
        HSEStatus = RCC->CR & RCC_CR_HSERDY;
        StartUpCounter++;
    } while ((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
        HSEStatus = (uint32_t)0x01;
    }
    else {
        HSEStatus = (uint32_t)0x00;
    }

    if (HSEStatus == (uint32_t)0x01) {
        /* Enable Prefetch Buffer */
        FLASH->ACR |= FLASH_ACR_PRFTBE;
        /* Flash 0 wait state */
        FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
        FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_0;

//@TODO - 1 Set the clock, (//) 주석 표시를 없애고 틀린 값이 있다면 제대로 된 값으로 수정하시오
        /* HCLK = SYSCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
        /* PCLK2 = HCLK / ?, use PPRE2 */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV2;//
        // have to be sysclock == pclk2.
        // pclk2 has to be 26, sysclock is 52
        // => divide 2
        /* PCLK1 = HCLK */
        RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

        /* Configure PLLs ------------------------------------------------------*/
        RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
        RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLMULL4); //
        // route going through prediv1 input -> mux(pllscr) -> pllmul
        // multiple 4 in PLLMUL register

        RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL | RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
        RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL13 | RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5); //
        //route going through prediv2 -> pll2mul -> mux(prediv1scr) -> prediv1
        // divide 5, multiple 13, divide 5
//@End of TODO - 1

        /* Enable PLL2 */
        RCC->CR |= RCC_CR_PLL2ON;
        /* Wait till PLL2 is ready */
        while ((RCC->CR & RCC_CR_PLL2RDY) == 0)
        {
        }
        /* Enable PLL */
        RCC->CR |= RCC_CR_PLLON;
        /* Wait till PLL is ready */
        while ((RCC->CR & RCC_CR_PLLRDY) == 0)
        {
        }
        /* Select PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
        /* Wait till PLL is used as system clock source */
        while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
        {
        }
        /* Select System Clock as output of MCO */
//@TODO - 2 Set the MCO port for system clock output
        RCC->CFGR &= ~(uint32_t)RCC_CFGR_MCO;
        RCC->CFGR |= (uint32_t)RCC_CFGR_MCO_SYSCLK; //
        // !< System clock selected as MCO source  - stm32f10x.h
//@End of TODO - 2
    }
    else {
        /* If HSE fails to start-up, the application will have wrong clock
        configuration. User can add here some code to deal with this error */
    }
    printf("SetSysClock end");
}

void RCC_Enable(void) {
    printf("RCC_Enable start");
//@TODO - 3 RCC Setting
    /*---------------------------- RCC Configuration -----------------------------*/
    /* GPIO RCC Enable  */
    /* UART Tx, Rx, MCO port */
    RCC->APB2ENR |= (uint32_t)RCC_APB2ENR_IOPAEN; //
    // port A => Tx, Rx exists
    /* USART RCC Enable */
    RCC->APB2ENR |= (uint32_t)RCC_APB2ENR_USART1EN; //
   /* User S1 Button RCC Enable */
    RCC->APB2ENR |= (uint32_t)RCC_APB2ENR_IOPCEN; //
    // key 1 botton on port c
    printf("RCC_Enable end");
}

void PortConfiguration(void) {
//@TODO - 4 GPIO Configuration
    /* Reset(Clear) Port A CRH - MCO, USART1 TX,RX*/
    printf("PortConfiguration start");
    GPIOA->CRH &= ~(
       (GPIO_CRH_CNF8 | GPIO_CRH_MODE8) |
       (GPIO_CRH_CNF9 | GPIO_CRH_MODE9) |
       (GPIO_CRH_CNF10 | GPIO_CRH_MODE10)
   );
    /* MCO Pin Configuration */
    GPIOA->CRH |= (GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8_1 | GPIO_CRH_MODE8_0); // 
    //cnf bit= 10(push-pull), mode bit = 11(max 50MHz) 
    /* USART Pin Configuration */
    GPIOA->CRH |= (GPIO_CRH_CNF9_1 | GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0
                   | GPIO_CRH_CNF10_1); //
    // port 9 => tx(output, push-pull, mode 11), port 10 => rx(input)
    
    /* Reset(Clear) Port C CRL - User S1 Button */
    GPIOC->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4); // 
    /* User S1 Button Configuration */
    GPIOC->CRL |= (GPIO_CRL_CNF4_1); // 
    printf("PortConfiguration end");
}

void UartInit(void) {
    printf("UartInit start");
    /*---------------------------- USART CR1 Configuration -----------------------*/
    /* Clear M, PCE, PS, TE and RE bits */
    USART1->CR1 &= ~(uint32_t)(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE);
    /* Configure the USART Word Length, Parity and mode ----------------------- */
    /* Set the M bits according to USART_WordLength value */
//@TODO - 6: WordLength : 8bit
    USART1->CR1 &= ~(uint32_t)(USART_CR1_M);  //
    // header file => default word length == 8(0x1000)
    
    /* Set PCE and PS bits according to USART_Parity value */
//@TODO - 7: Parity : None
    USART1->CR1 &= ~(uint32_t)(USART_CR1_PCE); //
    //to set parity none : nothing
    
    /* Set TE and RE bits according to USART_Mode value */
//@TODO - 8: Enable Tx and Rx
    USART1->CR1 |= (uint32_t)(USART_CR1_TE | USART_CR1_RE); // 

    /*---------------------------- USART CR2 Configuration -----------------------*/
    /* Clear STOP[13:12] bits */
    USART1->CR2 &= ~(uint32_t)(USART_CR2_STOP);
    /* Configure the USART Stop Bits, Clock, CPOL, CPHA and LastBit ------------*/
    USART1->CR2 &= ~(uint32_t)(USART_CR2_CPHA | USART_CR2_CPOL | USART_CR2_CLKEN);
    /* Set STOP[13:12] bits according to USART_StopBits value */
//@TODO - 9: Stop bit : 1bit
    USART1->CR2 &= ~(uint32_t)(USART_CR2_STOP); //

    /*---------------------------- USART CR3 Configuration -----------------------*/
    /* Clear CTSE and RTSE bits */
    USART1->CR3 &= ~(uint32_t)(USART_CR3_CTSE | USART_CR3_RTSE);
    /* Configure the USART HFC -------------------------------------------------*/
    /* Set CTSE and RTSE bits according to USART_HardwareFlowControl value */
//@TODO - 10: CTS, RTS : disable
    USART1->CR3 &= ~(uint32_t)(USART_CR3_CTSE | USART_CR3_RTSE); //


    /*---------------------------- USART BRR Configuration -----------------------*/
    /* Configure the USART Baud Rate -------------------------------------------*/
    /* Determine the integer part */
    /* Determine the fractional part */
//@TODO - 11: Calculate & configure BRR
    USART1->BRR |= 0xA94; // 
    // ppt p.21

    /*---------------------------- USART Enable ----------------------------------*/
    /* USART Enable Configuration */
//@TODO - 12: Enable UART (UE)
    USART1->CR1 |= USART_CR1_UE;// 
    printf("UartInit end");
}

void delay(void){
    int i = 0;
    for(i=0;i<1000000;i++);
}

void SendData(uint16_t data) {
   printf("SendData start\n");
    /* Transmit Data */
   USART1->DR = data;

   /* Wait till TC is set */
   while ((USART1->SR & USART_SR_TC) == 0);
   printf("SendData end\n");
}

int main() {
   printf("Hello world");
   int i;
   char msg[] = "Hello Team02\r\n";
   
    SysInit();
    SetSysClock();
    RCC_Enable();
    PortConfiguration();
    UartInit();
    
    // if you need, init pin values here
    
    
    while (1) {
        //@TODO - 13: Send the message when button is pressed
        if ((GPIOA->IDR & GPIO_IDR_IDR0) == 0)
        {
            i = 0;
            char* message = &msg[i];
            while (*message != '\0') {
                SendData(*message);
                message++;
            }
            delay();
        }
    }

}// end main