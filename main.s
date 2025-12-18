            UART Interrupt RGB Control System
;****************************************************************
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;Characters
SIM_SCGC5_PORTB EQU     (1<<10)         ; Enable clock to PORTB
LED_R_BIT       EQU     (1<<8)          ; PTB8  (red cathode)
LED_G_BIT       EQU     (1<<9)          ; PTB9  (green cathode)
LED_B_BIT       EQU     (1<<10)         ; PTB10 (blue cathode)
LED_ALL_BITS    EQU     (LED_R_BIT:OR:LED_G_BIT:OR:LED_B_BIT)
GPIO_MUX    EQU     (1<<8)
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
MAX_STRING	EQU  79
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue buffer contents
Q_REC_SZ    EQU   18  ;Queue management record
; Queue delimiters for printed output
Q_BEGIN_CH  EQU   '>'
Q_END_CH    EQU   '<'
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ICER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;22:PIT IRQ pending status
;12:UART0 IRQ pending status
NVIC_ICPR_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
;--PIT--------------------
PIT_IRQ_PRIORITY    EQU  0
NVIC_IPR_PIT_MASK   EQU  (3 << PIT_PRI_POS)
NVIC_IPR_PIT_PRI_0  EQU  (PIT_IRQ_PRIORITY << PIT_PRI_POS)
;--UART0--------------------
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;22:PIT IRQ mask
;12:UART0 IRQ mask
NVIC_ISER_PIT_MASK    EQU  PIT_IRQ_MASK
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PIT_LDVALn:  PIT load value register n
;31-00:TSV=timer start value (period in clock cycles - 1)
;Clock ticks for 0.01 s at ~24 MHz count rate
;0.01 s * ~24,000,000 Hz = ~240,000
;TSV = ~240,000 - 1
;Clock ticks for 0.01 s at 23,986,176 Hz count rate
;0.01 s * 23,986,176 Hz = 239,862
;TSV = 239,862 - 1
PIT_LDVAL_10ms  EQU  239861
;---------------------------------------------------------------
;PIT_MCR:  PIT module control register
;1-->    0:FRZ=freeze (continue'/stop in debug mode)
;0-->    1:MDIS=module disable (PIT section)
;               RTI timer not affected
;               must be enabled before any other PIT setup
PIT_MCR_EN_FRZ  EQU  PIT_MCR_FRZ_MASK
;---------------------------------------------------------------
;PIT_TCTRL:  timer control register
;0-->   2:CHN=chain mode (enable)
;1-->   1:TIE=timer interrupt enable
;1-->   0:TEN=timer enable
PIT_TCTRL_CH_IE  EQU  (PIT_TCTRL_TEN_MASK :OR: PIT_TCTRL_TIE_MASK)
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SCGC6
;1->23:PIT clock gate control (enabled)
;Use provided SIM_SCGC6_PIT_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
txrx_buff_size  EQU  80
;---------------------------------------------------------------
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
			EXPORT PutChar
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
	BL Init_UART0_IRQ
	LDR	R0, =QBuffer
	LDR R1, =QRecord
    MOVS R2, #Q_BUF_SZ
	BL InitQueue
	CPSIE	i
	
	LDR R0,=NewString
	MOVS R2,#NULL
	STRB R2, [R0,#0]	;initilize new string and put Null into it 
	
	
	LDR R0,=Count
	MOVS R2,#0
	STR R2, [R0,#0]	;initilize Count to 0
	
	LDR R0,=RunStopWatch
	MOVS R2,#1
	STRB R2, [R0,#0]	;initilize RunStopWatch to 1
	

	BL	Init_PIT_IRQ

Game_Start
	LDR R0, =INSTRUCT
	LDR R1, =INSTRUCT_end
	BL   PutStringSB
	BL NEWLINE
	BL GetChar        ;shows the prompt and wait for them to enter a character   
	
    LDR R0,=Round_Count
	MOVS R2,#1
	STR R2, [R0,#0]
	
    LDR R0,=User_Score
	MOVS R2,#0
	STR R2, [R0,#0]
	

    
ROUND_LOOP

	;turn light on
    LDR R0, =Count
    LDR R1, [R0, #0]
	BL SetColor
    MOVS R3, R0       ;save the remainder to compare later
	
	
	MOVS R0, #'>'
	BL PutChar

	
	LDR R0,=Count
	MOVS R2,#0
	STR R2, [R0,#0]      ;reset the timer count
	
	;put the max amount of time into r0 based on round
    LDR R1,=Round_Count
	LDR R0, [R1,#0]
    MOVS R2, #11
    SUBS R0, R2, R0
    

    
	BL GetCharWithTimer
	BCS Time_Failure      ;if C set then user ran out of time
	
    MOVS R0,R1
	CMP R0, #'a'
	BLO CharNotLowercase
	CMP R0,#'z'
	BHI CharNotLowercase
	ADDS R0,R0,#('A'-'a')
	
	BL PutChar      ;print the inputted character
    ;check if character is correct
CharNotLowercase
	CMP R0, #'G'
	BEQ Check_green
	CMP R0, #'R'
	BEQ Check_red
	CMP R0, #'W'
	BEQ Check_white
	CMP R0, #'B'
	BEQ Check_blue


	B Incorrect_Input
Check_green
	CMP R3, #2
	BEQ  Correct_Input
	B Incorrect_Input
Check_red
	CMP R3, #1
	BEQ  Correct_Input
	B Incorrect_Input
Check_white
	CMP R3, #3
	BEQ  Correct_Input
	B Incorrect_Input
Check_blue
	CMP R3, #0
	BEQ  Correct_Input
	B Incorrect_Input
	
	;handle wrong input case
Incorrect_Input
	LDR R0, =WRONG_STRING
	LDR R1, =WRONG_STRING_end
	BL   PutStringSB
	
	B Round_Done
	
    ;calculate the score to add
Correct_Input
	LDR R0, =CORRECT_STRING
	LDR R1, =CORRECT_STRING_end
	BL   PutStringSB
	MOVS R0, R4
	MOVS R1, R5
	BL   PutStringSB
	
    LDR R1,=Round_Count
	LDR R0, [R1,#0]
    LDR R1, =User_Score
    LDR R2, [R1, #0]
    ADDS R2, R2, R0
    STR R2, [R1, #0]
    B Round_Done
	
Time_Failure
	MOVS R0, #'X'
    BL PutChar
	LDR R0, =OOT_STRING
	LDR R1, =OOT_STRING_end
	BL   PutStringSB
	MOVS R0, R4
	MOVS R1, R5
	BL   PutStringSB
	
Round_Done
    BL NEWLINE
    
    ;increment round counter
    LDR R1,=Round_Count
    LDR R0, [R1,#0]
    CMP R0, #10         ;check if done game
    BEQ Restart_Game
    ADDS R0, R0, #1
    STR R0, [R1, #0]
    B ROUND_LOOP

Restart_Game
	LDR R0, =SCORE_STRING
	LDR R1, =SCORE_STRING_end
	BL   PutStringSB
    LDR R1, =User_Score
    LDR R0, [R1, #0]
    BL PutNumU              ;print the score
    BL NEWLINE
    BL Game_Start
    
    








;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP    ;main
				ALIGN
				LTORG
;>>>>> begin subroutine code <<<<<


;-------------------------------------------
;GetCharWithTimer
;inputs: R0 (time allowed)
;outputs: R1 (character to check) Set C flag if invalid 
;----------------------------------------------
GetCharWithTimer	PROC {R0-R13}
	PUSH {R0,R2-R5,LR}

	MOVS R6, R0                          ;save the timer value

PollRx
	LDR	R5,=Count
	LDR 	R1,[R5,#0]
	MOVS	R0, #100
	BL DIVU
	CMP     R0, R6
	BHI     Time_Out                     ;check if the time is up
	
    LDR     R1, =RxQRecord      ; pointer
    LDRB R2, [R1, #NUM_ENQD]     ; current count
    CMP R2, #0
    BEQ     PollRx                       ; loop until set

; Receive character into R1
    BL GetChar
    MOVS R1, R0
	MOVS R5, #0
    LSLS R5, R5, #1               ; clear carry (C=0)
	B  Done_GetCharWithTimer
	
Time_Out
	MOVS R5, #1
    LSRS R5, R5, #1               ; set carry (C=1)

Done_GetCharWithTimer
	POP {R0,R2-R5, PC}
	ENDP
    
    
;take in a number and change the color based off that
;and make the output something they can compare their input to
;input: R1 will be decimal (timer)
;output: R0 will be a decimal which is the remainder corresponding to the color
SetColor    PROC {R0-R13}
    PUSH    {LR}
    ; put in the numbers into DIVU
    MOVS    R0,#4            ; move 4 into R0 because we want to divide by 4
   
    ; its R1 / R0 = R2
    BL      DIVU             ; remainder is outputted into R1
   
    ; now there are only 4 possible remainders so set the color according to each one
    CMP     R1,#0            ; if its 0 set to blue
    BEQ     SetBlue
    CMP     R1,#1            ; if its 1 set to red
    BEQ     SetRed
    CMP     R1,#2            ; if its 2 set to green
    BEQ     SetGreen        
    CMP     R1,#3            ; if its 3 set to white
    BEQ     SetWhite
    B       Done
SetBlue
    BL      LED_Blue
    B       Done
SetRed
    BL      LED_Red
    B       Done
SetGreen
    BL      LED_Green
    B       Done
SetWhite
    BL      LED_White
    B       Done
Done
    MOVS    R0,R1          ; return remainder in R0
    POP     {PC}
    ENDP

Init_RGB_LED    PROC
        ; Configure PTB8, PTB9, PTB10 as GPIO (MUX=001)
        LDR     R0, =PORTB_PCR8
        LDR     R1, =GPIO_MUX         ; 0x0100, load via literal
        STR     R1, [R0]              ; PTB8
        STR     R1, [R0, #4]          ; PTB9
        STR     R1, [R0, #8]          ; PTB10

        ; Set them as outputs
        LDR     R0, =GPIOB_PDDR
        LDR     R1, [R0]
        LDR     R2, =LED_ALL_BITS     ; 0x0700
        ORRS    R1, R2
        STR     R1, [R0]

        ; Turn all LEDs off (active LOW, so write 1s to PSOR)
        LDR     R0, =GPIOB_PSOR
        LDR     R1, =LED_ALL_BITS
        STR     R1, [R0]

        BX      LR
        ENDP

; Turn all colors off
LED_Off PROC
        LDR     R0, =GPIOB_PSOR
        LDR     R1, =LED_ALL_BITS
        STR     R1, [R0]
        BX      LR
        ENDP

; Red only
LED_Red PROC {R0-R5,LR}
        PUSH    {R1, LR}                ; save caller return address

        BL      LED_Off             ; this can freely clobber LR now

        LDR     R0, =GPIOB_PCOR
        LDR     R1, =LED_R_BIT      
        STR     R1, [R0]
		
		LDR R4, =RED_STRING
		LDR R5, =RED_STRING_end
		
        POP     {R1, PC}                ; restore LR and return
        ENDP

; Green only
LED_Green PROC {R0-R5,LR}
        PUSH    {R1,LR}                ; save caller return address

        BL      LED_Off             ; this can freely clobber LR now

        LDR     R0, =GPIOB_PCOR
        LDR     R1, =LED_G_BIT      
        STR     R1, [R0]
		
		LDR R4, =GREEN_STRING
		LDR R5, =GREEN_STRING_end
		
        POP     {R1,PC}                ; restore LR and return
        ENDP

; Blue only
LED_Blue PROC {R0-R5,LR}
        PUSH    {R1,LR}                ; save caller return address

        BL      LED_Off             ; this can freely clobber LR now

        LDR     R0, =GPIOB_PCOR
        LDR     R1, =LED_B_BIT      ; 0x0400
        STR     R1, [R0]

		LDR R4, =BLUE_STRING
		LDR R5, =BLUE_STRING_end
		
        POP     {R1,PC}                ; restore LR and return
        ENDP
           
; White = Red + Green + Blue all ON (active low)
LED_White  PROC {R0-R5,LR}
        PUSH    {R1,LR}                ; save return address

        BL      LED_Off             ; turn everything off first

        LDR     R0, =GPIOB_PCOR
        LDR     R1, =LED_ALL_BITS   ; 0x0700 = R + G + B bits
        STR     R1, [R0]            ; drive all three low -> white

		LDR R4, =WHITE_STRING
		LDR R5, =WHITE_STRING_end
		
        POP     {R1,PC}
        ENDP
;****************************************************************
;* PutChar
;* Enqueues a character in R0 to the TxQ, safely (interrupt-protected).
;* Repeats until enqueue succeeds, then enables Tx interrupt.
;* Preserves all registers except R0 and PSR.
;****************************************************************

PutChar		PROC  {R0-R13}
    PUSH    {R1-R3, LR}      ; preserve registers well use

PutChar_Loop
    LDR     R1, =TxQRecord      ;
    CPSID   I                 ; disable interrupts
    BL      Enqueue           ; try to enqueue R0 into TxQueue
    CPSIE   I                 ; re-enable interrupts
    BCS     PutChar_Loop      ; if enqueue failed (queue full), retry

; Enable UART0 Transmit Interrupt (TIE bit in UART0_C2)
    LDR     R2, =UART0_BASE
	LDR 	R3, =UART0_C2_TI_RI
    STRB    R3, [R2, #UART0_C2_OFFSET]

    POP     {R1-R3, PC}
    ENDP


;****************************************************************
;* GetChar
;* Dequeues one byte from RxQ and returns it in R0.
;* Preserves all registers except R0 and PSR.
;****************************************************************
GetChar		PROC  {R0-R13}
    PUSH    {R1, LR}          ; preserve registers

GetChar_Loop
    LDR     R1, =RxQRecord      ; pointer
    CPSID   i                 ; mask interrupts
    BL      Dequeue           ; attempt to dequeue
    CPSIE   i                 ; unmask interrupts

    BCS     GetChar_Loop      ; if C=1 (queue empty), retry

    POP     {R1, PC}
    ENDP


GetStringSB PROC {R0-R13}
	PUSH {R2-R7,LR}
	MOVS R2, R0              ; R2 = pointer to string buffer
	SUBS R3, R1, #1          ; R3 = max chars - 1 (reserve space for null)
	MOVS R4, #0              ; R4 = counter (chars typed)

MainLoop_getString
	BL   GetChar             ; get current character in R0
	CMP  R0, #0x0D           ; carriage return?
	BEQ  GetSDone            ; if yes, done

	CMP  R0, #0x08           ; backspace pressed?
	BEQ  HandleBackspace     ; if yes, handle it

	CMP  R4, R3              ; buffer full?
	BGE  IgnoreChar          ; if full, ignore input

	; Store valid character
	STRB R0, [R2, R4]        ; store byte into buffer
	BL   PutChar             ; echo character
	ADDS R4, R4, #1          ; increment counter
	B    MainLoop_getString  ; repeat

HandleBackspace
	CMP  R4, #0              ; are there any chars to delete?
	BEQ  MainLoop_getString  ; if none, ignore backspace

	SUBS R4, R4, #1          ; reduce string length (delete last char)

	; move cursor back one space
	MOVS R0, #0x08
	BL   PutChar

	; overwrite with space
	MOVS R0, #' '
	BL   PutChar

	; move cursor back again
	MOVS R0, #0x08
	BL   PutChar

	B    MainLoop_getString  ; continue input


IgnoreChar
	B    MainLoop_getString


GetSDone
	BL   PutChar             ; echo carriage return
	MOVS R1, #0              ; null terminator
	STRB R1, [R2, R4]        ; store null byte
	MOVS R0, #0x0A           ; line feed
	BL   PutChar
	POP  {R2-R7,PC}
	ENDP







PutStringSB PROC {R0-R13}
	PUSH {R2-R7,LR}
	MOVS R2,R0			;pointer
	MOVS R4, #0			;counter
	SUBS R1,R1,#1		;buffer capacity
	
MainLoop_PutString
	LDRB R3, [R2,R4]	;load byte
	CMP R3, #0			;end if byte = 0
	BEQ PutSDone	
	MOVS R0,R3		
	BL PutChar			;print char
	ADDS R4,R4,#1		;increment counter
	CMP R4,R1			;check for buffer capacity 
	BGE PutSDone		;end loop
	B MainLoop_PutString

PutSDone
	POP {R2-R7,PC} 	
	ENDP


PutNumU PROC {R0-R13}

        PUSH    {R1-R5,LR}

        MOVS    R2, R0          ; copy value into R2

        CMP     R2, #0          ; check if zero
        BNE     GetPointer
        MOVS    R0, #'0'        ; ASCII '0'
        BL      PutChar
        B       DonePN

GetPointer
        MOV     R3, SP          ; save starting memory location
        MOV     R4, R3          ; R4 = current storage pointer 


MainLoop_PutNum
        MOVS    R1, R2
        MOVS    R0, #10
        BL      DIVU            ; quotient in R0, remainder in R1

        ADDS    R1, R1, #'0'   ; ASCII digit
        STRB    R1, [R4, #0]    ; store digit, advance buffer pointer
		ADDS 	R4,R4, #1

        MOVS    R2, R0          ; move quotient into R2
        CMP     R2, #0
        BNE     MainLoop_PutNum


PrintLoop
        SUBS    R4, R4, #1      ; move back one
        LDRB    R0, [R4,#0]        ; load digit
        BL      PutChar
        CMP     R4, R3          ; see if string is done
        BNE     PrintLoop

DonePN
        POP     {R1-R5,PC}
        ENDP	



DIVU 	PROC {R2-R14}
		PUSH {R2-R3,LR}	;Save inputs
		
		CMP R0,#0		;Check if zero
		BEQ ZERO
		
		MOVS R2, #0     ;set Quotient to 0
		
DIV_LOOP
		CMP R1,R0       ;while (Dividend >= Divisor)
		BLO DONE_DIVU
		SUBS R1,R1,R0	;Dividend = Dividend - Divisor
		ADDS R2,R2,#1	;Quotient = Quotient + 1
		B DIV_LOOP
DONE_DIVU 
		MOVS R0,R2
		MOVS R3,#0
		B ZERO_2
ZERO
		MOVS R3,#1

ZERO_2
		LSRS R3,R3,#1
		POP {R2-R3,PC}
		ENDP




NEWLINE  PROC {R0-R14}
	PUSH {R0,LR}
	MOVS R0,#0x0D
	BL PutChar
	MOVS R0, #0x0A
	BL PutChar
	POP {R0,PC}
	ENDP
	
	
; R0 = buffer start address
; R1 = address of queue record
; R2 = buffer size (character capacity)
;---------------------------------------------
InitQueue PROC {R0-R13}
    PUSH {R2-R5, LR}

    STR R0, [R1, #BUF_STRT]	; BUF_STRT = R0

    ADDS R3, R0, R2
    STR R3, [R1, #BUF_PAST]	;BUF_PAST = R0 + R2

    STR R0, [R1, #IN_PTR]	; IN_PTR = R0

    STR R0, [R1, #OUT_PTR]	; OUT_PTR = R0
 
    MOVS R3, #0
    STRB R3, [R1, #NUM_ENQD]	; NUM_ENQD = 0

    STRB R2, [R1, #BUF_SIZE]	; BUF_SIZE = R2

    POP {R2-R5, PC}
    ENDP



;====================================================
; Dequeue: Removes a character from the queue (R1)
; Returns dequeued character in R0
; Carry flag = 0 ? Success
; Carry flag = 1 ? Failure (queue empty)
;====================================================
Dequeue PROC {R0-R13}
    PUSH {R2-R5, LR}
    ; Check if queue empty
    LDRB R2, [R1, #NUM_ENQD]     ; current count
    CMP R2, #0
    BEQ Deq_Empty                 ; if empty, fail

    ; Get character from OUT_PTR
    LDR R3, [R1, #OUT_PTR]        ; R3 = OutPtr
    LDRB R0, [R3, #0]                 ; R0 = character

    ; Advance OUT_PTR
    ADDS R3, R3, #1
    LDR R4, [R1, #BUF_PAST]       ; R4 = BufPast
    CMP R3, R4
    BCC Deq_Cont
    LDR R3, [R1, #BUF_STRT]       ; wrap around if needed
Deq_Cont
    STR R3, [R1, #OUT_PTR]        ; update OutPtr

    ; Decrement NUM_ENQD
    SUBS R2, R2, #1
    STRB R2, [R1, #NUM_ENQD]

    ; Success: clear carry
    MOVS R5, #0
    LSLS R5, R5, #1               ; clear carry (C=0)
    B Deq_Done

    ; Failure: set carry
Deq_Empty
    MOVS R5, #1
    LSRS R5, R5, #1               ; set carry (C=1)

Deq_Done
    POP {R2-R5, PC}
    ENDP





; Enqueue: Adds a character (R0) into the queu	e (R1)
; Carry flag = 0 ? Success
; Carry flag = 1 ? Failure (queue full)
;====================================================

Enqueue PROC {R0-R13}
    PUSH {R0,R2-R6, LR}
    ; R1 = pointer to queue record
    LDRB R2, [R1, #NUM_ENQD]     ; current count
    LDRB R3, [R1, #BUF_SIZE]     ; max size (use LDRB since BUF_SIZE is 1 byte)
    CMP R2, R3
    BEQ Enq_Full                 ; if full, fail

    LDR R4, [R1, #IN_PTR]        ; R4 = InPtr
    STRB R0, [R4, #0]                ; store character at InPtr

    ; Advance IN_PTR
    ADDS R4, R4, #1
    LDR R5, [R1, #BUF_PAST]      ; R5 = BufPast
    CMP R4, R5
    BCC Enq_Cont
    LDR R4, [R1, #BUF_STRT]      ; wrap around if needed
Enq_Cont
    STR R4, [R1, #IN_PTR]        ; update InPtr

    ; Increment NUM_ENQD
    ADDS R2, R2, #1
    STRB R2, [R1, #NUM_ENQD]

    ; Success: clear carry
    MOVS R0, #0
    LSLS R0, R0, #1              ; clear carry (sets C=0)
    B Enq_Done

    ; Failure: set carry
Enq_Full
    MOVS R0, #1     ; set carry flag = 1
    LSRS R0, R0, #1    ;

Enq_Done
    POP {R0,R2-R6, PC}
    ENDP


;--------------------------------------------------
PrintQueue PROC {R0-R13}
    PUSH {R1-R6, LR}
    
    LDR R1, =QRecord        ; R1 = pointer to queue record
    LDRB R2, [R1, #NUM_ENQD] ; R2 = NUM_ENQD (byte)
    CMP R2, #0
    BEQ PQ_Done              ; If empty, nothing to print

    ; Setup pointers for traversal
    LDR R3, [R1, #OUT_PTR]   ; R3 = OUT_PTR (start)
    LDR R4, [R1, #BUF_SIZE]  ; R4 = BUF_SIZE
    LDR R5, [R1, #BUF_STRT]  ; R5 = BUF_STRT (base)
    LDR R6, [R1, #BUF_PAST]	 ; R6 = end adress

PQ_Loop
    LDRB R0, [R3,#0]            ; Load one byte from queue
    BL PutChar               ; Print character

    ADDS R3, R3, #1          ; Increment pointer
    CMP R3, R6               ; Reached end?
    BLO PQ_NoWrap
    MOV R3, R5               ; Wrap back to base
PQ_NoWrap

    SUBS R2, R2, #1          ; One fewer char left
    BNE PQ_Loop              ; Continue if not zero

PQ_Done
    POP {R1-R6, PC}
    ENDP



;--------------------------------------------------------
; PutNumHex
; Prints R0 as 8-digit hexadecimal (uppercase)
; Uses PutChar
;--------------------------------------------------------
PutNumHex PROC  {R0-R13}
    PUSH {R1-R4, LR}

    MOVS R1, #8          ; 8 nibbles to print
    MOV  R4, R0          ; copy original number into R4

HexLoop
    LSRS R2, R4, #28     ; get the top nibble
	MOVS R5, #0x0F
    ANDS R2, R5   ; mask to isolate nibble

    CMP R2, #9
    BLE IsDigit
    ADDS R2, R2, #('A' - 10)
    B NextDigit
IsDigit
    ADDS R2, R2, #'0'
NextDigit
    MOVS R0, R2
    BL PutChar

    LSLS R4, R4, #4      ; shift left by 4 to bring next nibble to top
    SUBS R1, R1, #1
    BNE HexLoop

    POP {R1-R4, PC}
    ENDP

; R0 = unsigned byte value
;-----------------------------------------------
PutNumUB PROC {R0-R13}
    PUSH {R1, LR}

    MOVS R1, #0xFF
    ANDS R0, R0, R1           ; mask to preserve only bit 0


    BL PutNumU

    POP {R1, PC}
    ENDP
		
;;------------------------------------------------------------
;; ReportStatus
;; Prints the queues InPtr, OutPtr, and NumEnqd values.
;; Output format example:
;; In=0x200000A0 Out=0x200000A4 Num=3
;;------------------------------------------------------------
;ReportStatus PROC {R0-R13}
    ;PUSH {R1-R3, LR}
    ;; Print "In=0x"
    ;LDR  R0, =IN_STRING
    ;LDR  R1, =IN_STRING_end
    ;BL   PutStringSB

    ;; Load and print InPtr (hex)
    ;LDR  R2, =QRecord
    ;LDR  R0, [R2, #IN_PTR]       ; R0 = InPtr value
    ;BL   PutNumHex

    ;; Print " Out=0x"
    ;LDR  R0, =OUT_STRING
    ;LDR  R1, =OUT_STRING_end
    ;BL   PutStringSB

    ;; Load and print OutPtr (hex)
    ;LDR  R0, [R2, #OUT_PTR]
    ;BL   PutNumHex

    ;; Print " Num="
    ;LDR  R0, =NUM_STRING
    ;LDR  R1, =NUM_STRING_end
    ;BL   PutStringSB

    ;; Load and print NumEnqd (unsigned byte)
    ;LDRB R0, [R2, #NUM_ENQD]
    ;BL   PutNumUB

    ;; Print newline (CR LF)
    ;MOVS R0, #0x0D      ; Carriage return
    ;BL   PutChar
    ;MOVS R0, #0x0A      ; Line feed
    ;BL   PutChar

    ;POP {R1-R3, PC}
    ;ENDP

;****************************************************************
;* UART0_ISR
;****************************************************************
UART0_ISR   PROC
    PUSH    {R4-R6, LR}

    LDR     R7, =UART0_BASE

; Check Transmit Interrupt 
    LDRB    R5, [R7, #UART0_C2_OFFSET]
    MOVS    R6, #UART0_C2_TIE_MASK
    TST     R5, R6
    BEQ     Check_Rx                    ; TIE not set, skip

    LDRB    R5, [R7, #UART0_S1_OFFSET]
    MOVS    R6, #UART0_S1_TDRE_MASK
    TST     R5, R6
    BEQ     Check_Rx                    ; TDRE not set, skip

; Try to dequeue next byte
    LDR     R1, =TxQRecord
    BL      Dequeue                     ; returns byte in R0, Carry=1 if empty
    BCS     Disable_TxInterrupt

    STRB    R0, [R7, #UART0_D_OFFSET]   ; send byte
    B       Check_Rx

Disable_TxInterrupt
    LDRB    R5, [R7, #UART0_C2_OFFSET]
    MOVS    R6, #UART0_C2_TIE_MASK
    BICS    R5, R6                      ; clear TIE bit
    STRB    R5, [R7, #UART0_C2_OFFSET]

;  Check Receive Interrupt 
Check_Rx
    LDRB    R5, [R7, #UART0_S1_OFFSET]
    MOVS    R6, #UART0_S1_RDRF_MASK
    TST     R5, R6
    BEQ     ISR_Exit

    LDRB    R0, [R7, #UART0_D_OFFSET]   ; read received byte
    LDR     R1, =RxQRecord
    BL      Enqueue                     ; enqueue (ignore if full)

ISR_Exit
    POP     {R4-R6, PC}
    ENDP





;****************************************************************
;* Init_UART0_IRQ
;****************************************************************
Init_UART0_IRQ	PROC  {R0-R13}
    PUSH    {R0-R7, LR}
    ;Set     UART0 IRQ priority
    LDR R0,=UART0_IPR
    LDR R1,=NVIC_IPR_UART0_MASK
    LDR R2,=NVIC_IPR_UART0_PRI_3
    LDR R3,[R0,#0]
    BICS R3,R3,R1
    ORRS R3,R3,R2
    STR R3,[R0,#0]
;Clear any pending UART0 interrupts
    LDR R0,=NVIC_ICPR
    LDR R1,=NVIC_ICPR_UART0_MASK
    STR R1,[R0,#0]
;Unmask UART0 interrupts
    LDR R0,=NVIC_ISER
    LDR R1,=NVIC_ISER_UART0_MASK
    STR R1,[R0,#0]


; Initialize Receive and Transmit Queues
    ; Rx Queue: 80-byte buffer
    LDR     R0, =RxQBuffer
    LDR     R1, =RxQRecord
    MOVS    R2, #txrx_buff_size
    BL      InitQueue

    ; Tx Queue: 80-byte buffer
    LDR     R0, =TxQBuffer
    LDR     R1, =TxQRecord
    MOVS    R2, #txrx_buff_size
    BL      InitQueue


; Init UART0
    LDR     R0, =SIM_SOPT2
    LDR     R1, =SIM_SOPT2_UART0SRC_MASK
    LDR     R2, [R0, #0]
    BICS    R2, R2, R1
    LDR     R1, =SIM_SOPT2_UART0SRC_MCGFLLCLK
    ORRS    R2, R2, R1
    STR     R2, [R0, #0]

; Set UART0 for external connection
    LDR     R0, =SIM_SOPT5
    LDR     R1, =SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
    LDR     R2, [R0, #0]
    BICS    R2, R2, R1
    STR     R2, [R0, #0]

; Enable UART0 module clock
    LDR     R0, =SIM_SCGC4
    LDR     R1, =SIM_SCGC4_UART0_MASK
    LDR     R2, [R0, #0]
    ORRS    R2, R2, R1
    STR     R2, [R0, #0]

; Enable PORT B module clock
    LDR     R0, =SIM_SCGC5
    LDR     R1, =SIM_SCGC5_PORTB_MASK
    LDR     R2, [R0, #0]
    ORRS    R2, R2, R1
    STR     R2, [R0, #0]
    BL      Init_RGB_LED

; Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
    LDR     R0, =PORTB_PCR2
    LDR     R1, =PORT_PCR_SET_PTB2_UART0_RX
    STR     R1, [R0, #0]

; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
    LDR     R0, =PORTB_PCR1
    LDR     R1, =PORT_PCR_SET_PTB1_UART0_TX
    STR     R1, [R0, #0]

; Disable UART0 receiver and transmitter
    LDR     R0, =UART0_BASE
    MOVS    R1, #UART0_C2_T_R
    LDRB    R2, [R0, #UART0_C2_OFFSET]
    BICS    R2, R2, R1
    STRB    R2, [R0, #UART0_C2_OFFSET]

; Set UART0 for 9600 baud, 8N1 protocol
    MOVS    R1, #UART0_BDH_9600
    STRB    R1, [R0, #UART0_BDH_OFFSET]

    MOVS    R1, #UART0_BDL_9600
    STRB    R1, [R0, #UART0_BDL_OFFSET]

    MOVS    R1, #UART0_C1_8N1
    STRB    R1, [R0, #UART0_C1_OFFSET]

    MOVS    R1, #UART0_C3_NO_TXINV
    STRB    R1, [R0, #UART0_C3_OFFSET]

    MOVS    R1, #UART0_C4_NO_MATCH_OSR_16
    STRB    R1, [R0, #UART0_C4_OFFSET]

    MOVS    R1, #UART0_C5_NO_DMA_SSR_SYNC
    STRB    R1, [R0, #UART0_C5_OFFSET]

    MOVS    R1, #UART0_S1_CLEAR_FLAGS
    STRB    R1, [R0, #UART0_S1_OFFSET]

    MOVS    R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
    STRB    R1, [R0, #UART0_S2_OFFSET]

; Enable UART0 receiver and transmitter
    MOVS    R1, #UART0_C2_T_RI
    STRB    R1, [R0, #UART0_C2_OFFSET]
	
; Done  restore registers
    POP     {R0-R7, PC}
    ENDP
		
;****************************************************************
;* Subroutine: Init_PIT_IRQ
;****************************************************************
Init_PIT_IRQ   PROC  {R0-R14}
    PUSH    {R0-R5, LR}             ; Preserve working registers

;---------------------------------------------------------------
; Enable clock for PIT module (SIM_SCGC6 bit 23)
;---------------------------------------------------------------
    LDR     R0, =SIM_SCGC6
    LDR     R1, =SIM_SCGC6_PIT_MASK
    LDR     R2, [R0, #0]
    ORRS    R2, R2, R1
    STR     R2, [R0, #0]

;---------------------------------------------------------------
; Disable PIT Timer 0 (TEN=0)
;---------------------------------------------------------------
    LDR     R0, =PIT_CH0_BASE
    LDR     R1, =PIT_TCTRL_TEN_MASK
    LDR     R2, [R0, #PIT_TCTRL_OFFSET]
    BICS    R2, R2, R1
    STR     R2, [R0, #PIT_TCTRL_OFFSET]

;---------------------------------------------------------------
; Configure NVIC for PIT interrupts
;---------------------------------------------------------------
; Priority configuration
    LDR     R0, =PIT_IPR
    LDR     R1, =NVIC_IPR_PIT_MASK
    LDR     R2, =NVIC_IPR_PIT_PRI_0
    LDR     R3, [R0, #0]
    BICS    R3, R3, R1
    ORRS    R3, R3, R2
    STR     R3, [R0, #0]

; Clear any pending PIT interrupts
    LDR     R0, =NVIC_ICPR
    LDR     R1, =NVIC_ICPR_PIT_MASK
    STR     R1, [R0, #0]

; Unmask PIT interrupts
    LDR     R0, =NVIC_ISER
    LDR     R1, =NVIC_ISER_PIT_MASK
    STR     R1, [R0, #0]

;---------------------------------------------------------------
; Enable PIT module (MDIS=0) and freeze in debug (FRZ=1)
;---------------------------------------------------------------
    LDR     R0, =PIT_BASE
    LDR     R1, =PIT_MCR_EN_FRZ
    STR     R1, [R0, #PIT_MCR_OFFSET]

;---------------------------------------------------------------
; Set PIT Timer 0 period for 0.01 s at 23.986176 MHz
;---------------------------------------------------------------
    LDR     R0, =PIT_CH0_BASE
    LDR     R1, =PIT_LDVAL_10ms
    STR     R1, [R0, #PIT_LDVAL_OFFSET]

;---------------------------------------------------------------
; Enable PIT Timer 0 and its interrupt
;---------------------------------------------------------------
    LDR     R1, =PIT_TCTRL_CH_IE
    STR     R1, [R0, #PIT_TCTRL_OFFSET]

    POP     {R0-R5, PC}
    ENDP


;****************************************************************
;* PIT_ISR
;* Periodic Interrupt Timer 0 Interrupt Service Routine
;****************************************************************
PIT_ISR   PROC  {R0-R14}
    PUSH {R0-R3, LR}

    ; Check if RunStopWatch != 0
    LDR     R0, =RunStopWatch
    LDRB    R1, [R0, #0]
    CMP     R1, #0
    BEQ     SkipIncrement

    ; Increment Count
    LDR     R0, =Count
    LDR     R2, [R0, #0]
    ADDS    R2, R2, #1
    STR     R2, [R0, #0]

SkipIncrement
    ;Clear PIT Channel 0 interrupt
    LDR R0,=PIT_CH0_BASE
    LDR R1,=PIT_TFLG_TIF_MASK
    STR R1,[R0,#PIT_TFLG_OFFSET]

    POP {R0-R3, PC}
    ENDP





	
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendSV (PendableSrvReq)
                                      ;   pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 transfer 
                                      ;   complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:FTFA command complete/
                                      ;   read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:(reserved)
            DCD    Dummy_Handler      ;26:SPI0
            DCD    Dummy_Handler      ;27:(reserved)
            DCD    UART0_ISR      ;28:UART0_ISR (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR      ;38:PIT
            DCD    Dummy_Handler      ;39:(reserved)
            DCD    Dummy_Handler      ;40:(reserved)
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:(reserved)
            DCD    Dummy_Handler      ;46:PORTA
            DCD    Dummy_Handler      ;47:PORTB
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
INSTRUCT	DCB		"Type the first letter of the correspondong color that flashes on the board. If the color is red type r or R. Enter any character to begin. "	,NULL
INSTRUCT_end

CORRECT_STRING DCB ":  Correct--", NULL
CORRECT_STRING_end

WRONG_STRING  DCB ":  Wrong", NULL
WRONG_STRING_end

OOT_STRING  DCB ":  Out of time--", NULL
OOT_STRING_end

Thank_You	DCB		"Thank you. Goodbye!", NULL
Thank_You_end

SCORE_STRING  DCB  "Final score: ", NULL
SCORE_STRING_end

GREEN_STRING DCB "Color was green",NULL
GREEN_STRING_end

RED_STRING DCB "Color was red",NULL
RED_STRING_end

BLUE_STRING DCB "Color was blue",NULL
BLUE_STRING_end

WHITE_STRING DCB "Color was white",NULL
WHITE_STRING_end
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
NewString	SPACE	MAX_STRING
QBuffer	SPACE Q_BUF_SZ
		ALIGN
QRecord SPACE Q_REC_SZ
	ALIGN
TxQBuffer	SPACE	txrx_buff_size
	ALIGN
TxQRecord	SPACE 	Q_REC_SZ
	ALIGN
RxQBuffer	SPACE	txrx_buff_size
	ALIGN
RxQRecord	SPACE	Q_REC_SZ

RunStopWatch	SPACE	1
	ALIGN
Count		SPACE	4  
    ALIGN
Round_Count SPACE   4
    ALIGN
User_Score  space   4
;>>>>>   end variables here <<<<<
            ALIGN

            END
