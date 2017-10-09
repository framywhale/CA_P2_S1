//First version without passing conditions at each stage!!

//Did not pass control signals from stage to stage!!

//Each wire has a wire variable defined with a suffix telling which stage it's in

//alu.v has changed (ALUop->4 bits)
//reg_file.v did not change

module mycpu_top(
    input            clk,
    input            resetn,            //low active

    output           inst_sram_en,
    output  [ 3:0]   inst_sram_wen,
    output  [31:0]   inst_sram_addr,
    output  [31:0]   inst_sram_wdata,
    input   [31:0]   inst_sram_rdata,
    
    output           data_sram_en,
    output  [ 3:0]   data_sram_wen,
    output  [31:0]   data_sram_addr,
    output  [31:0]   data_sram_wdata,
    input   [31:0]   data_sram_rdata,

    //debug interface
    output  [31:0]   debug_wb_pc,
    output  [3 :0]   debug_wb_rf_wen,
    output  [4 :0]   debug_wb_rf_wnum,
    output  [31:0]   debug_wb_rf_wdata
);

    wire rst;


    reg  [31:0] PC;
    wire [31:0] PC_added;    //PC+4
    wire [31:0] inst_addr;
    wire [31:0] Jump_addr;
    wire [31:0] Branch_addr;
    
    
    reg  [31:0]      PC_IF_ID;
    reg  [31:0] PC_added_IF_ID;
    reg  [31:0]            IR;
    
//Datapath    
    wire [ 4:0]        rs, rt;    //Respectively assigned to IR[25:21] and IR[21:16]
    wire [31:0]     SgnExtend;
    wire [31:0]  RegRdata1_ID;
    wire [31:0]  RegRdata2_ID;
    wire [31:0]   J_target_ID;
    wire [31:0]   PC_added_ID;
    wire                 Zero;
    wire [31:0]  SgnExtend_LF;
//Control Signals
    wire [ 3:0]      ALUop_ID;
    wire [ 1:0]      PCSrc_ID;
    wire [ 1:0]     RegDst_ID;
    wire [ 3:0]   RegWrite_ID;
    wire [ 3:0]   MemWrite_ID;
    wire [ 1:0]    ALUSrcA_ID;
    wire [ 1:0]    ALUSrcB_ID;
    wire             MemEn_ID;
    wire              JSrc_ID;
    wire          MemToReg_ID;
//Debug
    wire [31:0]   PC_ID;
    
    
    
    
//Datapath    
    reg  [31:0]  PC_added_ID_EX;  //Did not use wire between 
    reg  [31:0]  J_target_ID_EX;
    reg  [31:0] RegRdata1_ID_EX;
    reg  [31:0] RegRdata2_ID_EX;
    reg  [ 4:0] RegWaddr1_ID_EX;  //IR[20:16]
    reg  [ 4:0] RegWaddr2_ID_EX;  //IR[15:11]
//Control Signals
    reg  [ 3:0]     ALUop_ID_EX;
    reg  [ 1:0]    RegDst_ID_EX;
    reg  [ 3:0]  RegWrite_ID_EX;
    reg  [ 3:0]  MemWrite_ID_EX;
    reg  [ 1:0]   ALUSrcA_ID_EX;
    reg  [ 1:0]   ALUSrcB_ID_EX;
    reg             MemEn_ID_EX;
    reg          MemToReg_ID_EX;
//Debug
    reg  [31:0]  PC_ID_EX;

//Datapath   
    wire [31:0]  PC_added_EX; 
    wire [31:0] RegRdata1_EX;
    wire [31:0] RegRdata2_EX;
    wire [ 4:0] RegWaddr1_EX;
    wire [ 4:0] RegWaddr2_EX;
    wire [31:0]  J_target_EX;
    wire [31:0] ALUB, ALUA, ALUResult;
    wire [31:0] sa_EX;
    wire [ 4:0] RegWaddr_EX;
    wire        AOverflow, AZero, ACarryOut;
//Control Signals
    wire [ 3:0]      ALUop_EX;
    wire [ 1:0]     RegDst_EX;
    wire [ 3:0]   RegWrite_EX;
    wire [ 3:0]   MemWrite_EX;
    wire [ 1:0]    ALUSrcA_EX;
    wire [ 1:0]    ALUSrcB_EX;
    wire             MemEn_EX;
    wire          MemToReg_EX;
//Debug
    wire [31:0]   PC_EX;
        





//Datapath    
    reg           ALUZero_EX_MEM;
    reg  [31:0] ALUResult_EX_MEM;
    reg  [31:0]  MemWdata_EX_MEM;
    reg  [31:0]  RegWaddr_EX_MEM;
//Control Signals    
    reg  [ 3:0]  MemWrite_EX_MEM;
    reg          MemToReg_EX_MEM;
    reg             MemEn_EX_MEM;
    reg  [ 3:0]  RegWrite_EX_MEM;
//Debug
    reg  [31:0]  PC_EX_MEM;
    
    
    
    
//Datapath    
	wire [ 4:0]  RegWaddr_MEM;
	wire [31:0] ALUResult_MEM;
	wire [31:0]  MemWdata_MEM;
//Control Signals
    wire [ 3:0]  MemWrite_MEM;
    wire         MemToReg_MEM;
    wire            MemEn_MEM;
    wire [ 3:0]  RegWrite_MEM;
//Debug
    wire [31:0]  PC_MEM;


    
//Datapath    
    reg  [31:0]  MemRdata_MEM_WB;
    reg  [31:0] ALUResult_MEM_WB;
    reg  [ 4:0]  RegWaddr_MEM_WB; 
//Control Signals
    reg         MemToReg_MEM_WB;
    reg  [ 3:0] RegWrite_MEM_WB;
//Debug
    reg  [31:0] PC_MEM_WB;    


    
//Datapath    
    wire [ 4:0]  RegWaddr_WB;        //assigned to RegWaddr_MEM_WB
    wire [31:0]  MemRdata_WB;
    wire [31:0] ALUResult_WB;
    wire [31:0]  RegWdata_WB;
//Control Signals
    wire       MemToReg_WB;    
    wire [3:0] RegWrite_WB;
//Debug
    wire [31:0] PC_WB;
    
    assign rst = ~resetn;
    
    assign Jump_addr = JSrc_ID ? J_target_EX : RegRdata1_EX;   //Not yet decided
    
    assign inst_sram_en    = 1;
    assign inst_sram_wen   = 4'b0000;
    assign inst_sram_addr  = inst_addr;
    assign inst_sram_wdata = 32'd0;
    
    assign data_sram_en    =     MemEn_MEM;
    assign data_sram_wen   =  MemWrite_MEM;
    assign data_sram_addr  = ALUResult_MEM;
    assign data_sram_wdata =  MemWdata_MEM;
    
    assign SgnExtend    = {{16{IR[15]}},IR[15:0]};
    assign PC_added_ID  = PC_added_IF_ID;
    assign J_target_ID  = {{PC_IF_ID[31:28]},{IR[25:0] << 2}};
    assign rs           = IR[25:21];
    assign rt           = IR[20:16];
    assign SgnExtend_LF = SgnExtend << 2;
    
    assign PC_ID = PC_IF_ID;
///////////////////////////////////////////////////

    
    assign PC_added_EX  =  PC_added_ID_EX;
    assign RegRdata1_EX = RegRdata1_ID_EX;
    assign RegRdata2_EX = RegRdata2_ID_EX;
    assign  J_target_EX =  J_target_ID_EX;
    assign RegWaddr1_EX = RegWaddr1_ID_EX;
    assign RegWaddr2_EX = RegWaddr2_ID_EX;
    assign ALUA = ALUSrcA_EX ? PC_added_EX : RegRdata1_EX;
    assign sa_EX = {{27{1'b0}}, RegWaddr2_ID_EX};
///////////////////////////////////////////////////////
    assign ALUop_EX     =    ALUop_ID_EX;
    assign RegDst_EX    =   RegDst_ID_EX;
    assign RegWrite_EX  = RegWrite_ID_EX;
    assign MemWrite_EX  = MemWrite_ID_EX;
    assign ALUSrcA_EX   =  ALUSrcA_ID_EX;
    assign ALUSrcB_EX   =  ALUSrcB_ID_EX;
    assign MemEn_EX     =    MemEn_ID_EX;
    assign MemToReg_EX  = MemToReg_ID_EX;    
    
    assign PC_EX = PC_ID_EX;


   
    assign ALUResult_MEM = ALUResult_EX_MEM;
    assign MemWdata_MEM  =  MemWdata_EX_MEM;
    assign RegWaddr_MEM  =  RegWaddr_EX_MEM;
////////////////////////////////////////////////////////
    assign MemWrite_MEM  = MemWrite_EX_MEM;
    assign MemToReg_MEM  = MemToReg_EX_MEM;
    assign MemEn_MEM     =    MemEn_EX_MEM;    
    assign RegWrite_MEM  = RegWrite_EX_MEM;
    
    assign PC_MEM = PC_EX_MEM;
    
    
    
    
    assign MemRdata_WB  =  MemRdata_MEM_WB;
    assign ALUResult_WB = ALUResult_MEM_WB;
    assign RegWaddr_WB  =  RegWaddr_MEM_WB; 
/////////////////////////////////////////////////////////
    assign MemToReg_WB  =  MemToReg_MEM_WB;
    assign RegWrite_WB  =  RegWrite_MEM_WB;
    
    assign PC_WB = PC_MEM_WB;
    
    
    assign RegWdata_WB = MemToReg_WB ? MemRdata_WB : ALUResult_WB;
    
    assign debug_wb_pc       =       PC_WB;
    assign debug_wb_rf_wen   = RegWrite_WB;
    assign debug_wb_rf_wnum  = RegWaddr_WB;
    assign debug_wb_rf_wdata = RegWdata_WB;
    
    
    
//////////////////////////////////////////////////////
//Instruction Fetch State    
    always @ (posedge clk) begin
    if (rst)
    	PC <= 32'hbfc00000;
    else begin
        PC             <= PC_added;
        PC_IF_ID       <= inst_addr;
        PC_added_IF_ID <= PC_added;
        IR             <= inst_sram_rdata;
        end
    end
    
//////////////////////////////////////////////////////
//Instruction Decode State
    always @ (posedge clk) begin
        PC_added_ID_EX <= PC_added_ID;
        J_target_ID_EX <= J_target_ID;
        RegRdata1_ID_EX <= RegRdata1_ID;
        RegRdata2_ID_EX <= RegRdata2_ID;
        RegWaddr1_ID_EX <= IR[20:16];
        RegWaddr2_ID_EX <= IR[15:11];     //Data

        ALUop_ID_EX    <=    ALUop_ID;
        RegDst_ID_EX   <=   RegDst_ID;
        RegWrite_ID_EX <= RegWrite_ID;
        MemWrite_ID_EX <= MemWrite_ID;
        ALUSrcA_ID_EX  <=  ALUSrcA_ID;
        ALUSrcB_ID_EX  <=  ALUSrcB_ID;
        MemEn_ID_EX    <=    MemEn_ID;
        MemToReg_ID_EX <= MemToReg_ID;    //Signals
        
    end

///////////////////////////////////////////////////////
//Instruction Execution State
    always @ (posedge clk) begin
        ALUResult_EX_MEM <= ALUResult;
        MemWdata_EX_MEM  <= RegRdata2_EX;
        RegWaddr_EX_MEM  <= RegWaddr_EX;
        ALUZero_EX_MEM   <= AZero;         //Data
        
        MemWrite_EX_MEM  <= MemWrite_EX;
        MemToReg_EX_MEM  <= MemToReg_EX;
        MemEn_EX_MEM     <=    MemEn_EX;   
        RegWrite_EX_MEM  <= RegWrite_EX;   //Signals
    end

///////////////////////////////////////////////////////
//MEM State
    always @ (posedge clk) begin
        MemRdata_MEM_WB  <= data_sram_rdata;
        ALUResult_MEM_WB <= ALUResult_MEM;
        RegWaddr_MEM_WB  <=  RegWaddr_MEM;  //Data
        
        MemToReg_MEM_WB  <=  MemToReg_MEM;  
        RegWrite_MEM_WB  <=  RegWrite_MEM;  //Signals
    end



///////////////////////////////////////////////////////
//Instantiation of each module
    MUX_3_32 PCSrc_MUX(
        .Src1   (PC),
        .Src2   (Jump_addr),
        .Src3   (Branch_addr),
        .op     (PCSrc_ID),
        .Result (inst_addr)
    );
    Adder PC_ADDER(
    	.A      (inst_addr),
    	.B      (    32'd4),
    	.Result (PC_added)
    );
    reg_file RegFile(
        .clk    (clk),
        .rst    (rst),
        .waddr  (RegWaddr_WB),
        .raddr1 (rs),
        .raddr2 (rt),
        .wen    (RegWrite_WB),
        .wdata  (RegWdata_WB),
        .rdata1 (RegRdata1_ID),
        .rdata2 (RegRdata2_ID)
    );
    Control_Unit Control(
        .rst       (rst),
        .zero      (Zero),
        .op        (IR[31:26]),
        .func      (IR[ 5: 0]),
        .MemEn     (MemEn_ID),
        .Jsrc      (JSrc_ID),
        .MemToReg  (MemToReg_ID),
        .ALUop     (ALUop_ID),
        .PCSrc     (PCSrc_ID),
        .RegDst    (RegDst_ID),
        .RegWrite  (RegWrite_ID),
        .MemWrite  (MemWrite_ID),
        .ALUSrcA   (ALUSrcA_ID),
        .ALUSrcB   (ALUSrcB_ID)
    );
    Zero_Cal Branch_Determination(
        .A      (RegRdata1_ID),
        .B      (RegRdata2_ID),
        .Result (Zero)
    );
    MUX_3_32 ALUA_MUX(
        .Src1   (PC_added_EX),
        .Src2   (RegRdata1_EX),
        .Src3   (sa_EX),
        .op     (ALUSrcA_EX),
        .Result (ALUA)
    );
    MUX_3_32 ALUB_MUX(
        .Src1   (RegRdata2_EX),
        .Src2   (SgnExtend),
        .Src3   (32'd4),
        .op     (ALUSrcB_EX),
        .Result (ALUB)
    );
    MUX_3_5 RegWaddr_MUX(
        .Src1   (RegWaddr1_EX),
        .Src2   (RegWaddr2_EX),
        .Src3   (5'b11111),
        .op     (RegDst_EX),
        .Result (RegWaddr_EX)
    );
    ALU ALU(
        .A        (ALUA), 
        .B        (ALUB), 
        .ALUop    (ALUop_EX), 
        .Overflow (AOverflow), 
        .CarryOut (ACarryOut), 
        .Zero     (AZero),
        .Result   (ALUResult)
    );
    Adder Branch_addr_Adder(
        .A      (PC_added_ID),
        .B      (SgnExtend_LF),
        .Result (Branch_addr)
    );
    
    
endmodule    


/////////////////////////////////////////////////////////
//Three input MUX of thirty two bits

module MUX_3_32(
	input  [31:0] Src1,
	input  [31:0] Src2,
	input  [31:0] Src3,
	input  [ 1:0] op,
	output [31:0] Result
);
    wire [31:0] and1, and2, and3, op1, op1x, op0, op0x;
    
	assign op1  = {32{ op[1]}};
    assign op1x = {32{~op[1]}};
    assign op0  = {32{ op[0]}};
    assign op0x = {32{~op[0]}};
    assign and1 = Src1   & op1x & op0x;
    assign and2 = Src2   & op1x & op0;
    assign and3 = Src3   & op1  & op0x;
    
    assign Result = and1 | and2 | and3;
endmodule

////////////////////////////////////////////////////////
//Instantiated form of ALU module(ALUop assigned to add)    
module Adder(
	input  [31:0] A,    
	input  [31:0] B,
    output [31:0] Result
);
    ALU adder(
        .A      (A),
        .B      (B),
        .ALUop  (4'b0010),   //ADD
        .Result (Result)
    );
endmodule

//////////////////////////////////////////////////////////
//Three input MUX of five bits
module MUX_3_5(
    input  [4:0] Src1,
    input  [4:0] Src2,
    input  [4:0] Src3,
    input  [1:0] op,
    output [4:0] Result
);
    wire [4:0] and1, and2, and3, op1, op1x, op0, op0x;
    
	assign op1  = {5{ op[1]}};
    assign op1x = {5{~op[1]}};
    assign op0  = {5{ op[0]}};
    assign op0x = {5{~op[0]}};
    assign and1 = Src1   & op1x & op0x;
    assign and2 = Src2   & op1x & op0;
    assign and3 = Src3   & op1  & op0x;
    
    assign Result = and1 | and2 | and3;     
endmodule

//////////////////////////////////////////////////////////
//Instantiated form of ALU module(ALUop assigned to sub)
module Zero_Cal(
    input [31:0] A,
    input [31:0] B,
    output       Result
);
    ALU Zero(
        .A    (A),
        .B    (B),
        .op   (4'b0110),   //SUB
        .Zero (Result)
    );
endmodule
