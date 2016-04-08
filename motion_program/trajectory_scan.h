; *****************************************************************************************
; Variables
; *****************************************************************************************

#define RootVar 4000

; EPICS Required Variables

#define Status              P(RootVar + 1)          ; Status of motion program for EPICS - 0: Initialised, 1: Active, 2: Idle, 3: Error
#define Abort               P(RootVar + 2)          ; Abort trigger for EPICS
#define Axes                P(RootVar + 3)          ; An int between 1 and 511 specifying which axes to use
#define BufferLength        P(RootVar + 4)          ; Length of a single buffer e.g. AX, AY...
#define TotalPoints         P(RootVar + 5)          ; Total number of points scanned through

#define CurrentIndex        P(RootVar + 6)          ; Current index position in buffer
#define CurrentBuffer       P(RootVar + 7)          ; Current buffer specifier - 0: A, 1: B

#define BufferAdr_A         P(RootVar + 8)          ; Start index of buffer A
#define BufferAdr_B         P(RootVar + 9)          ; Start index of buffer B
#define CurrentBufferAdr    P(RootVar + 10)         ; A or B buffer address

#define BufferFill_A        P(RootVar + 11)         ; Fill level of buffer A
#define BufferFill_B        P(RootVar + 12)         ; Fill level of buffer B
#define CurrentBufferFill   P(RootVar + 13)         ; A or B buffer fill level
#define PrevBufferFill      P(RootVar + 14)         ; Fill level of previous buffer

; Motion Program Variables

#define X_Axis              P(RootVar + 101)        ; Specifiers for what axes are to be used
#define Y_Axis              P(RootVar + 102)
#define Z_Axis              P(RootVar + 103)
#define U_Axis              P(RootVar + 104)
#define V_Axis              P(RootVar + 105)
#define W_Axis              P(RootVar + 106)
#define A_Axis              P(RootVar + 107)
#define B_Axis              P(RootVar + 108)
#define C_Axis              P(RootVar + 109)

#define Time                P(RootVar + 110)        ; Current coordinate values
#define X_Coord             P(RootVar + 111)
#define Y_Coord             P(RootVar + 112)
#define Z_Coord             P(RootVar + 113)
#define U_Coord             P(RootVar + 114)
#define V_Coord             P(RootVar + 115)
#define W_Coord             P(RootVar + 116)
#define A_Coord             P(RootVar + 117)
#define B_Coord             P(RootVar + 118)
#define C_Coord             P(RootVar + 119)
#define User                P(RootVar + 120)
#define VelMode             P(RootVar + 121)
#define Next_Time           (RootVar + 122)

#define Prev_X              P(RootVar + 131)        ; Previous coordinate values
#define Prev_Y              P(RootVar + 132)
#define Prev_Z              P(RootVar + 133)
#define Prev_U              P(RootVar + 134)
#define Prev_V              P(RootVar + 135)
#define Prev_W              P(RootVar + 136)
#define Prev_A              P(RootVar + 137)
#define Prev_B              P(RootVar + 138)
#define Prev_C              P(RootVar + 139)

#define X_Vel               P(RootVar + 141)        ; Move velocities
#define Y_Vel               P(RootVar + 142)
#define Z_Vel               P(RootVar + 143)
#define U_Vel               P(RootVar + 144)
#define V_Vel               P(RootVar + 145)
#define W_Vel               P(RootVar + 146)
#define A_Vel               P(RootVar + 147)
#define B_Vel               P(RootVar + 148)
#define C_Vel               P(RootVar + 149)

; Address-Based Variables

#define BlankAddress        M4015                   ; Address storing a zero for unused axes to point to
BlankAddress->D:$3FFFF,0,48

#define Next_Time_N         M4000                   ; Next coordinate values
#define Next_X              M4001
#define Next_Y              M4002
#define Next_Z              M4003
#define Next_U              M4004
#define Next_V              M4005
#define Next_W              M4006
#define Next_A              M4007
#define Next_B              M4008
#define Next_C              M4009
#define Next_User           M4010
#define NextVelMode         M4011

NextVelMode->X:$3FFFF,4,4                           ; Set initial pointers and type
Next_User->X:$3FFFF,0,4
Next_Time_N->Y:$3FFFF,0,24
Next_X->L:$3FFFF,0,48
Next_Y->L:$3FFFF,0,48
Next_Z->L:$3FFFF,0,48
Next_U->L:$3FFFF,0,48
Next_V->L:$3FFFF,0,48
Next_W->L:$3FFFF,0,48
Next_A->L:$3FFFF,0,48
Next_B->L:$3FFFF,0,48
Next_C->L:$3FFFF,0,48

#define Time_Adr            M4020                   ; Pointers to Next_* coordinate addresses
#define X_Adr               M4021
#define Y_Adr               M4022
#define Z_Adr               M4023
#define U_Adr               M4024
#define V_Adr               M4025
#define W_Adr               M4026
#define A_Adr               M4027
#define B_Adr               M4028
#define C_Adr               M4029
#define User_Adr            M4030
#define VelMode_Adr         M4031

Time_Adr->Y$4FA0,0,24                               ; Assignments for pointers to M address locations
X_Adr->Y:$4FA1,0,24                                 ; M0 = $4000 -> M4000 = $4FA0
Y_Adr->Y:$4FA2,0,24
Z_Adr->Y:$4FA3,0,24
U_Adr->Y:$4FA4,0,24
V_Adr->Y:$4FA5,0,24
W_Adr->Y:$4FA6,0,24
A_Adr->Y:$4FA7,0,24
B_Adr->Y:$4FA8,0,24
C_Adr->Y:$4FA9,0,24
User_Adr->Y:$4FAA,0,24
VelMode_Adr->Y:$4FAB,0,24

#define RTE                 M4041
RTE->Y:$203F,22
