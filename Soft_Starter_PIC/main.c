
// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

#include <xc.h>
#include <string.h>
#include <stdio.h>
#include "lcd.h"

#define _XTAL_FREQ 8000000 // Usado como base para função __delay_ms()

// CONFIG1L
#pragma config PLLDIV = 3       // PLL Prescaler Selection bits (No prescale (4 MHz oscillator input drives PLL directly))
#pragma config CPUDIV = OSC1_PLL2// System Clock Postscaler Selection bits ([Primary Oscillator Src: /1][96 MHz PLL Src: /2])
#pragma config USBDIV = 1       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes directly from the primary oscillator block with no postscale)

// CONFIG1H
#pragma config FOSC = INTOSC_HS    // Oscillator Selection bits (EC oscillator, CLKO function on RA6 (EC))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOR = ON         // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 1         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer Enable bit (WDT enabled)
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF      // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = OFF      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = OFF       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

char auxTimer = 0;
int cont=0;
int contTimer0=65459;//65536 - ajuste 65461 - ajuste pic pratica 65459
int contTimer3=65459;//65536 - ajuste 65461 - ajuste pic pratica 65459
char auxBorda=0;
char auxTriacAcionado=0;
int TempoDesaceleracao=5;//Tempo em segundos
int TempoAceleracao=10;//Tempo em segundos
int TempoAtual=0;
int TempoChaveamentoInicial=55;//Tempo inicial de ciclo, multiplicado por 0,1ms - Inicia com 8,1ms -- 55
int CiclosIncrementoAcel;
int CiclosIncrementoDesac;
int ContCiclosIncremento=0;
char MotorLigado=0;
char IniciaAceleracao=0;
char IniciaDesaceleracao=0;
int contCorrente=0;
char passagemZero=0;
float picoCorrente=0;
float CorrenteRMS=0;
    
#define Fase1 PORTEbits.RE0
#define Fase2 PORTEbits.RE1
#define Fase3 PORTEbits.RE2
#define SinaleiroMotorLigado PORTCbits.RC0
#define ParteMotor PORTCbits.RC1
#define ParaMotor PORTAbits.RA1
#define IncAceleracao PORTAbits.RA2
#define DecAceleracao PORTAbits.RA3
#define IncDesaceleracao PORTAbits.RA4
#define DecDesaceleracao PORTAbits.RA5

void DelayFor18TCY( void )
{
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
  Nop();
}
void DelayPORXLCD (void)
{
  Delay1KTCYx(30); // Delay of 15ms
                   // Cycles = (TimeDelay * Fosc) / 4
                   // Cycles = (15ms * 16MHz) / 4
                   // Cycles = 60,000
                   // Para 8Mhz-30
  return;
}
void DelayXLCD (void)
{
  Delay1KTCYx(10); // Delay of 5ms
                   // Cycles = (TimeDelay * Fosc) / 4
                   // Cycles = (5ms * 16MHz) / 4
                   // Cycles = 20,000
                   // Para 8Mhz-10
  return;
}

void Mydelay(int tempo)
{
   int i;
   for (i=0;i<tempo;i++)
   {
      __delay_ms(1);
   }
}
 
 void InicializaLCD(void)
 {
    OpenXLCD(FOUR_BIT & LINES_5X7);
    while(BusyXLCD()); //Aguarda controlador do display livre
    WriteCmdXLCD(0x01); //Limpa display
    while(BusyXLCD());
    WriteCmdXLCD(CURSOR_OFF & BLINK_OFF);
 }
 
  void parada_forcada (void) { //Para rotação do motor - Sobrecorrente
    MotorLigado = 0;
    SinaleiroMotorLigado = 0;
    TMR0IE = 0;
    INTCON = 0b00000000; //Desativa interrupções
    INT1IE = 0;
    INT2IE = 0;
    Fase1 = 0;
    Fase2 = 0;
    Fase3 = 0;
    SetDDRamAddr(0x00);//Posição do cursor no display
    putsXLCD("   PARADA POR   ");
    SetDDRamAddr(0x40);//Posição do cursor no display
    putsXLCD("SOBRECORRENTE!!!");
}
  
 void func_timer0_acel (void) { //Estouro do TIMER0
    cont++;
    TMR0IF=0; //Reset da interrupção causada pelo TIMER0
    
    contCorrente++;
    
    if (cont>=TempoAtual && auxTriacAcionado==0){ //0,1ms cada incremento
        cont=0;
        ContCiclosIncremento++;
        if (ContCiclosIncremento>=CiclosIncrementoAcel){
            ContCiclosIncremento = 0;
            TempoAtual--;
        }
        if (TempoAtual<=31){ //25
            SinaleiroMotorLigado = 1;
            MotorLigado = 1;
            TempoAtual = 31;
        }
        auxTriacAcionado=1;
        Fase1 = 1;
    }
    else if (cont>=1 && auxTriacAcionado==1){
        cont = 0;
        auxTriacAcionado=0;
        Fase1 = 0;
        TMR0IE=0;
    }   
}
 
  void func_timer0_desacel (void) { //Estouro do TIMER0
    cont++;
    TMR0IF=0; //Reset da interrupção causada pelo TIMER0
    
    if (cont>=TempoAtual && auxTriacAcionado==0){ //0,1ms cada incremento
        cont=0;
        ContCiclosIncremento++;
        if (ContCiclosIncremento>=CiclosIncrementoDesac){
            ContCiclosIncremento = 0;
            TempoAtual++;
        }
        if (TempoAtual>=70){
            TempoAtual = TempoChaveamentoInicial;
            MotorLigado = 0;
            SinaleiroMotorLigado = 0;
            TMR0IE=0;
            INTCON = 0b01000000; //Desativa interrupções
            INT1IE = 0;
            INT2IE = 0;
        }
        else{
            auxTriacAcionado=1;
            Fase1 = 1;
        }
    }
    else if (cont>=1 && auxTriacAcionado==1){
        cont = 0;
        auxTriacAcionado=0;
        Fase1 = 0;
        TMR0IE=0;
    }
}
  
 void func_timer2 (void) { //Estouro do TIMER2
    static int cont2=0;
    static char auxTriac2Acionado=0;
    cont2++;
    TMR2IF=0;
    
    if (cont2>=TempoAtual && auxTriac2Acionado==0){ //0,1ms cada incremento
        cont2=0;
        auxTriac2Acionado=1;
        Fase2 = 1;
    }
    else if (cont2>=1 && auxTriac2Acionado==1){
        cont2 = 0;
        auxTriac2Acionado=0;
        Fase2 = 0;
        TMR2IE=0; //Desativa o TIMER2
    }
    
}
 
 void func_timer3 (void) { //Estouro do TIMER3
    static cont3=0;
    static char auxTriac3Acionado=0;
    cont3++;
    TMR3IF=0;
    
    if (cont3>=TempoAtual && auxTriac3Acionado==0){ //0,1ms cada incremento
        cont3=0;
        auxTriac3Acionado=1;
        Fase3 = 1;
    }
    else if (cont3>=1 && auxTriac3Acionado==1){
        cont3 = 0;
        auxTriac3Acionado=0;
        Fase3 = 0;
        TMR3IE=0; //Desativa o TIMER3
    }
    
}
 
void interrupt altaPrioridade(void)
{
    if(INT0IF==1 && INT0IE==1) { //Interrupção gerada por INT0
        passagemZero++;
        INT0IF = 0;
        TMR0IE=1;
    }
    else if(INT1IF==1 && INT1IE==1) { //Interrupção gerada por INT1
        INT1IF = 0;
        TMR2IE = 1;
    }
    else if(INT2IF==1 && INT2IE==1) { //Interrupção gerada por INT2
        INT2IF = 0;
        TMR3IE = 1;
    }
    
    if(TMR0IF==1 && TMR0IE==1){ //Verifica estouro do TIMER0 e chama sua função
        TMR0=contTimer0; //Zera contador TMR0
        if (IniciaAceleracao==1 && IniciaDesaceleracao==0)
            func_timer0_acel();
        else if (IniciaAceleracao==0 && IniciaDesaceleracao==1)
            func_timer0_desacel();
    }
    if(TMR2IF==1 && TMR2IE==1){ //Verifica estouro do TIMER2 e chama sua função
        func_timer2();
    }
    if(TMR3IF==1 && TMR3IE==1){ //Verifica estouro do TIMER3 e chama sua função
        TMR3=contTimer3; //Zera contador TMR3
        func_timer3();
    }
}

 void main(void){
    OSCCON = 0b01110111; //Habilita oscilador interno
    TRISE = 0; // Setando port E como saída
    LATE = 0;
    TRISC = 0b00000010; // Setando port C como entradas e saídas
    LATC = 0;
    ADCON1 = 0x0F;
    TRISA = 255; // Setando port A como entrada
    TempoAtual = TempoChaveamentoInicial;
    char StringTempo[10],StringCorrente[10];
    float LeituraAN0;
    char DelayBT1,DelayBT2,DelayBT3,DelayBT4;
    char BloqBT1,BloqBT2,BloqBT3,BloqBT4;
    int AcelAnterior,DesacelAnterior;
    
    // Configuração dos timers
    OpenTimer0 (TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_2);
    
    OpenTimer2(TIMER_INT_ON & T2_POST_1_10 & T2_PS_1_4); //Prescaler 1:16 * Postscaler 1:10
    PR2 = 4;
    
    OpenTimer3(TIMER_INT_ON & T3_16BIT_RW & T1_SOURCE_INT & T1_SYNC_EXT_OFF & T1_PS_1_2);
    TMR3=contTimer3;
    
    //Inicia com todos os timers desabilitados
    TMR1IE = 0;
    TMR2IE = 0;
    TMR3IE = 0;
    
    //Prioridade alta das interrupções geradas por estouro dos timers
    TMR1IP = 1;
    TMR2IP = 1;
    TMR3IP = 1;
    
    INTCON2bits.TMR0IP=1;
     
    /* Interrupções*/
    /*Interrupção (alta prioridade):*/
    INTCON2bits.INTEDG0=1; //Habilita interrupção pela RB0 como borda de subida
    INTCON2bits.RBPU=1; //Habilita Pull Ups porta B
    INTCON2bits.INTEDG1=1; //Habilita interrupção pela RB1 como borda de subida
    INTCON2bits.INTEDG2=1; //Habilita interrupção pela RB2 como borda de subida
    
    INT1IP=1;
    INT1IE=1;
    INT2IP=1;
    INT2IE=1;
    
    //Desabilita interrupções
    INTCON = 0b01000000; //Habilita as interrupções externas e global e zera flag da interrupção
                         //Declarado dessa forma para não gerar conflito pela função INTCONbits com outras bibliotecas
                         //Habilita interrupção pela RB0 por mudança de estado
                         //Inicializa Timer0
    
    //Inicializa entrada analógica para leitura de corrente
    OpenADC(ADC_FOSC_64 & // ADC_FOSC_64: Clock de conversão do A/D igual a
            // FAD = FOSC/64 = 48MHz/64 = 750kHz
            // Desta forma, TAD=1/FAD = 1,33us.
            ADC_RIGHT_JUST & // ADC_RIGHT_JUST: Resultado da conversão ocupará os
            // bits menos significativos dos registradores ADRESH e ADRESL
            ADC_12_TAD, // ADC_12_TAD: Determina o tempo de conversão de uma
            // palavra de 10-bits. Neste caso será igual a 12*TAD = 12*1,33us = 16us.
            ADC_CH0 & // ADC_CH0: selecionar o canal no qual será realizada a //conversão, neste caso o AN0.
            ADC_INT_OFF & // ADC_INT_OFF: Desabilita a interrupção de término de conversão.
            ADC_REF_VDD_VSS, // ADC_VREFPLUS_VDD: Determina o VDD (+5V) como tensão de //referência positiva (VREF+) e o VSS (0V)como tensão de
            //referência negativa (VREF-).
            ADC_1ANA); // Configura o pino AN0 como Entradas Analógicas
    
    //Controle display LCD
    InicializaLCD();
    SetDDRamAddr(0x00);//Posição do cursor no display
    putsXLCD("Corrente: 0.00A");
    SetDDRamAddr(0x40);//Posição do cursor no display
    sprintf(StringTempo, "Acl:%d", TempoAceleracao);
    putsXLCD(StringTempo);
    putsXLCD("s  ");
    SetDDRamAddr(0x48);//Posição do cursor no display
    sprintf(StringTempo, "Des:%d", TempoDesaceleracao);
    putsXLCD(StringTempo);
    putsXLCD("s  ");
        
    while(1){
        
        //-------- Leitura de corrente através de AN0 --------

        /*SetDDRamAddr(0x08);//Posição do cursor no display
        sprintf(StringCorrente, "%.2fv ", LeituraAN0);
        putsXLCD(StringCorrente);*/
        /*
        if (LeituraAN0>=4){ //Sobrecorrente
            parada_forcada();
            while(1); //Trava execucao do programa
        }*/
        
        if (contCorrente>=10){
            contCorrente=0;
            SetChanADC(ADC_CH0);
            ConvertADC();
            while (BusyADC());
            while (BusyUSART());
            LeituraAN0 = ReadADC();
            LeituraAN0 = (LeituraAN0*5)/(3*1024); //10 bits; Valor máximo=1024->5V
            if (LeituraAN0>picoCorrente){
                picoCorrente=LeituraAN0;
            }
        }
        
        if (passagemZero>=50){
            passagemZero=0;
            CorrenteRMS=picoCorrente/1.4142;
            SetDDRamAddr(0x0A);//Posição do cursor no display
            sprintf(StringCorrente, "%.2fA ", CorrenteRMS);
            putsXLCD(StringCorrente);
            if (CorrenteRMS>1 && MotorLigado==1){ //Sobrecorrente
                parada_forcada();
                while(1); //Trava execucao do programa
            }
            picoCorrente=0;
        }
        
        //-------- Atualiza dados de tempos de aceleração e desaceleração --------
        CiclosIncrementoAcel = (TempoAceleracao*1000)/(8.3333333*24);//Incrementos de 0,1ms
        CiclosIncrementoDesac = (TempoDesaceleracao*1000)/(8.3333333*39);//Incrementos de 0,1ms
        
        //--------- Alteração dos valores de aceleração e desaceleração --------
        if (IncAceleracao==0){
            DelayBT1 = 0;
            BloqBT1 = 0;
        }
        if (DecAceleracao==0){
            DelayBT2 = 0;
            BloqBT2 = 0;
        }
        if (IncDesaceleracao==0){
            DelayBT3 = 0;
            BloqBT3 = 0;
        }
        if (DecDesaceleracao==0){
            DelayBT4 = 0;
            BloqBT4 = 0;
        }
        
        if (IncAceleracao==1)
            DelayBT1++;
        if (DecAceleracao==1)
            DelayBT2++;
        if (IncDesaceleracao==1)
            DelayBT3++;
        if (DecDesaceleracao==1)
            DelayBT4++;
        
        if (DelayBT1>=10 && BloqBT1==0 && TempoAceleracao<99){
            TempoAceleracao++;
            BloqBT1 = 1;
        }
        if (DelayBT2>=10 && BloqBT2==0 && TempoAceleracao>1){
            TempoAceleracao--;
            BloqBT2 = 1;
        }
        if (DelayBT3>=10 && BloqBT3==0 && TempoDesaceleracao<99){
            TempoDesaceleracao++;
            BloqBT3 = 1;
        }
        if (DelayBT4>=10 && BloqBT4==0 && TempoDesaceleracao>1){
            TempoDesaceleracao--;
            BloqBT4 = 1;
        }
        
        if (TempoAceleracao!=AcelAnterior){
            AcelAnterior = TempoAceleracao;
            SetDDRamAddr(0x44);//Posição do cursor no display
            sprintf(StringTempo, "%ds ", TempoAceleracao);
            putsXLCD(StringTempo);
        }
        if (TempoDesaceleracao!=DesacelAnterior){
            DesacelAnterior = TempoDesaceleracao;
            SetDDRamAddr(0x4C);//Posição do cursor no display
            sprintf(StringTempo, "%ds  ", TempoDesaceleracao);
            putsXLCD(StringTempo);
        }
        //--------- Inicia partida ou parada do motor --------
        
        if (MotorLigado==0 && ParteMotor==1){
            ContCiclosIncremento = 0;
            IniciaAceleracao = 1;
            IniciaDesaceleracao = 0;
            INTCON = 0b11010000; //Habilita interrupções
            INT1IE = 1;
            INT2IE = 1;
        }
        else if (MotorLigado==1 && ParaMotor==1){
            ContCiclosIncremento = 0;
            IniciaAceleracao = 0;
            IniciaDesaceleracao = 1;
        }
    }
}