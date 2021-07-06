#include <p18F4550.h>         		// NESTA LINHA DEVE SER INSERIDO O DISPOSITIVO.
#define  _XTAL_FREQ 20000000		// Definição da frequência do oscilador externo.

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                         Configurações para gravação                   *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#pragma config FOSC = HS            // Oscilador de alta velocidade
#pragma config PBADEN = OFF         // Desabilita conversor A/D no PORTB
#pragma config PWRT = ON            // Habilita Power-up Timer
#pragma config WDT = OFF            // Watch-dog Timer desabilitado
#pragma config LVP = OFF            // Desabilita programação por baixa tensão
#pragma config DEBUG = OFF          // Desabilita modo de DEGUB

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                INCLUDES DAS FUNÇÕES DE PERIFÉRICOS DO PIC             *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include <pwm.h>                  //PWM library functions
#include <adc.h>                  //ADC library functions
#include <timers.h>               //Timer library functions
#include <delays.h>               //Delay library functions
#include <i2c.h>                  //I2C library functions
#include <stdlib.h>               //Library functions
#include <stdio.h>                //Output functions
#include <usart.h>                //USART library functions
#include <math.h>
#include <string.h>
#include <biblioteca_lcd_2x16.h>

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                       PROTÓTIPO DE FUNÇÕES                            *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void Delay_ms(unsigned int z);
void Delay_us(unsigned int z);
void Delay_seg(unsigned int z);
void inicializa_adc(void);
unsigned int leitura_adc(void);
void IHM(void);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                           Constantes internas                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//A definição de constantes facilita a programação e a manutenção.

#define const pi 3.1415

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                 Definição e inicialização das variáveis                 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Neste bloco estão definidas as variáveis globais do programa.

unsigned int i=0, cont = 0, sirene = 0;
char buffer[16];
float sp = 0, temperatura = 0;
signed int x=0, x_int=0, x_dec=0, SP = 0, SP_int=0, SP_dec=0;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                ENTRADAS                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// As entradas devem ser associadas a nomes para facilitar a programação e
//futuras alterações do hardware.

#define ERB1 PORTBbits.RB1 //Botão mudar tela IHM
#define ERB2 PORTBbits.RB2 //Botão incrementar variável IHM
#define ERB3 PORTBbits.RB3 //Botão decrementar variável IHM
#define ERB4 PORTBbits.RB4 //Botão desabilita alarme de emergência IHM
#define ERB5 PORTBbits.RB5 //Botão que simula chave liga/desliga do sistema
#define ERB6 PORTBbits.RB6 //Botão que simula sensor de nível ALTO
#define ERB7 PORTBbits.RB7 //Botão que simula sensor de nível BAIXO

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                                 SAÍDAS                              *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
// AS SAÍDAS DEVEM SER ASSOCIADAS A NOMES PARA FACILITAR A PROGRAMAÇÃO E
// FUTURAS ALTERAÇÕES DO HARDWARE.

#define SRA2 PORTAbits.RA2 //Saída para o sinaleiro de emergência
#define SRA3 PORTAbits.RA3 //Saída para o alarme de emergência
#define SRC0 PORTCbits.RC0 //Saída para o controle da válvula superior
#define SRC1 PORTCbits.RC1 //Saída para o controle da válvula inferior
#define SRC2 PORTCbits.RC2 //Saída para o controle do motor
#define SRC4 PORTCbits.RC4 //Saída para o controle do aquecedor


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                            Função Principal                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void main(){ // Função principal

   PORTA = 0x00;                       //Clear PORTA
   PORTB = 0x00;                       //Clear PORTB
   PORTC = 0x00;                       //Clear PORTC
   PORTD = 0x00;                       //Clear PORTD
   PORTE = 0x00;                       //Clear PORTE
  
   LATA = 0x00;                        //Clear LATA
   LATB = 0x00;                        //Clear LATB
   LATC = 0x00;                        //Clear LATC
   LATD = 0x00;                        //Clear LATD
   LATE = 0x00;                        //Clear LATE

   TRISA = 0b00000000;                 //CONFIG DIREÇÃO DOS PINOS PORTA
   TRISB = 0b00111111;                 //CONFIG DIREÇÃO DOS PINOS PORTB => 0: saída , 1: entrada
   TRISC = 0b00000000;                 //CONFIG DIREÇÃO DOS PINOS PORTC
   TRISD = 0b00000000;                 //CONFIG DIREÇÃO DOS PINOS PORTD
   TRISE = 0b00000000;                 //CONFIG DIREÇÃO DOS PINOS PORTE      
   
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *                            Rotina Principal                         *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
   inicializa_adc();
   lcd_inicia(0x28,0x0C,0x06);

	while(1){ //Enquanto o sistema estiver energizado
		
		IHM(); //Acesso à IHM

		while(ERB5 == 1){ //Enquanto o sistema estiver ligado
				
			IHM(); //Acesso à IHM

			while(ERB5 == 1 && (ERB6 == 1 && ERB7 == 0)){ //Enquanto houver erro (Sensor HIGH ligado e LOW desligado)
				SRC0 = 0; //Desliga a válvula superior
				SRC1 = 0; //Desliga a válvula inferior
				SRC2 = 0; //Desliga o motor
				SRC4 = 0; //Desliga o aquecedor
				IHM(); //Acesso à IHM
				SRA2 = 1; //Liga o sinaleiro
				if(sirene == 0) SRA3 = 1; //Liga a sirene se ainda não foi desligada manualmente
				if(ERB4 == 1){ //Se o botão da sirene for pressionado
					SRA3 = 0; //Desliga a sirene
					sirene = 1; //Impede que a sirene ligue denovo 
				}	
			}
			sirene = 0; //Reinicia a sirene
			SRA2 = 0; //Desliga o sinaleiro	
			SRA3 = 0; //Desliga a sirene
				
			if(ERB6 == 1){ //Se o sensor HIGH estiver ligado
				while(ERB5 == 1 && ERB7 == 1){ //Enquanto o sensor LOW estiver em 1
					IHM(); //Acesso à IHM
					SRC0 = 0; //Desliga a válvula superior
					if(temperatura < sp){ //Se a temperatura do liquido for menor que a referência
						IHM(); //Acesso à IHM
						SRC4 = 1; //Liga o aquecedor
						SRC2 = 1; //Liga o motor
						SRC1 = 0; //Fecha a válvula inferior
					}else while(ERB5 == 1 && (temperatura >= sp && ERB7 == 1)){ /*Se não, enquanto a temperatura do liquido for maior 
																			 	  que a referência e o sensor LOW estiver ligado */
						i = 2; //Trava a IHM na tela 2
						IHM(); //Acesso à IHM
						SRC4 = 0; //Desliga o aquecedor
		    			SRC2 = 0; //Desliga o motor
						SRC1 = 1; //Abre a válvula inferior
					}
				}
			}else while(ERB5 == 1 && ERB6 == 0){ //Se não, enquanto o sensor HIGH estiver em 0
					IHM(); //Acesso à IHM
					SRC1 = 0; //Fecha a válvula inferior
					SRC0 = 1; //Abre a válvula superior
			}
		}

		SRC0 = 0; //Fecha a válvula superior
		SRC1 = 0; //Fecha a válvula inferior
		SRC2 = 0; //Desliga o motor
		SRC4 = 0; //Desliga o aquecedor		
		 
	} 
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Funcao:      Delay_ms
 * Entrada:     *z , valor de 0 a 65535.
 * Saída:       Nenhuma (void)
 * Descrição:   Essa função é capaz de gerar 65535ms,de temporização.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void Delay_ms(unsigned int z){
    int y;
    for(y=0;y<z;y++)
    {
         Delay100TCYx(50);
	     
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Funcao:      Delay_us
 * Entrada:     *z , valor de 0 a 65535.
 * Saída:       Nenhuma (void)
 * Descrição:   Essa função é capaz de gerar 65535us,de temporização.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
  void Delay_us(unsigned int z){
    int y;
    for(y=0;y<z;y++)
    {
        Delay1TCY();Delay1TCY();Delay1TCY();
        Delay1TCY();Delay1TCY();
    }
}
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Funcao:      Delay_seg
 * Entrada:     *z , valor de 0 a 65535.
 * Saída:       Nenhuma (void)
 * Descrição:   Essa função é capaz de gerar 65535s,de temporização.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void Delay_seg(unsigned int z){
    int y;
    for(y=0;y<z;y++)
    {
        Delay10KTCYx(250);
        Delay10KTCYx(250);   
    }
}

void inicializa_adc(void){
	 TRISA =  0b00000011;     // Configura RA0 (AN1) como entrada
	 ADCON0 = 0b00000000;     // Configura referência de tensão (Vdd e Vss) e habilita AN0 (entrada do ADC)
	 ADCON1 = 0b00001101;     // Configura entradas analógicas (apenas os pinos AN0 e AN1)
	 ADCON2 = 0b10010000;     // Configura o tempo de aquisição (vide datasheet)
	 ADCON0bits.ADON = 1;     // Liga módulo ADC
}

unsigned int leitura_adc(void){
     unsigned int leitura;
	 ADCON0bits.CHS = 0b0001;				// Escolhe o canal analógico que será lido
	 ADCON0bits.GO = 1;                     // Inicia a aquisição
 	 while(ADCON0bits.DONE){}    // Aguarda o fim da aquisição (quando bit DONE = 0)
 	 leitura = ADRESH;                      // Parte mais significativa do valor lido
 	 leitura = ((leitura << 8)|ADRESL );    // Une a parte mais e menos significativa em um valor de 16 bits
	 return leitura;
}

void IHM(void){
			temperatura = leitura_adc() / 2.046; //Lê a temperatura do sensor
	
			x = temperatura*100; 						 
			x_int = x/100;    						 
			x_dec = x%100; if(x_dec < 0)x_dec * -1; //Divide a temperatura em parte inteira e decimal


			switch(i){ //Controle de telas

				case 0:{ //Primeira tela
					lcd_posicao(1,1);
					imprime_string_lcd("   TEMPFLUID    "); //Printa "TEMPFLUID" na primeira tela
					break;
				}

				case 1:{ //Segunda tela
					lcd_posicao(1,1);
					imprime_string_lcd("Temp do Liquido"); //Printa "Temp do Liquido" na segunda tela
					lcd_posicao(2,1);

						if(ERB2 == 1) sp+=0.5; //Incrementa a referência de temperatura em 0.5
 						if(sp>50) sp=50;
						Delay_ms(50);

						if(ERB3 == 1) sp-=0.5; //Decrementa a referência de temperatura em 0.5
						if(sp<0) sp=0;
						Delay_ms(50);

						SP = sp*100; //Divide a referência em parte inteira e decimal
						SP_int = SP/100;
						SP_dec = SP%100; if(SP_dec<0) SP_dec*=-1;

						sprintf(buffer,"Ref = %02d.%01d C        ",SP_int,SP_dec/10); //Salva "Ref = " e a erferência no buffer
						imprime_buffer_lcd(buffer,16); //Printa o buffer na tela

						break;
				}

				case 2:{ //Terceira tela
					lcd_posicao(1,1);
					imprime_string_lcd("Temp: "); //Printa "Temp: " na terceira tela
					sprintf(buffer,"%01d.%02d C      ",x_int,x_dec); //Salva a temperatura no buffer
					imprime_buffer_lcd(buffer,16); //Printa o buffer na tela
					lcd_posicao(2,1);
					sprintf(buffer,"Ref = %02d.%01d C         ",SP_int,SP_dec/10); //Salva a referência no buffer
					imprime_buffer_lcd(buffer,16); //Printa o buffer na tela
				
					break;	
				}
		}
		
			if(ERB1 == 1){ //Botão troca de tela
				i++;
			    lcd_limpa_tela();
		    }
			if(i>2) i=1;
			while(ERB1 == 1){}
			
}
