module my1bitmux (output out, input i0, i1, sel); // header & ports
  wire n_sel, x1, x2; // internal nets
  or (out, x1, x2); // output
  and (x1, i0, n_sel); // i0 & (~sel)
  and (x2, i1, sel); // i1 & sel
  not (n_sel, sel); // invert sel
endmodule

module my1bithalfadder(output wire S, output wire C, input A, B);
  wire ix;
  my1bitmux m1(C, 1'b0, A, B);
  my1bitmux m2(ix, 1'b1, 1'b0, B);
  my1bitmux m3(S, B, ix, A);
endmodule

module my1bitfulladder(output wire S, output wire C, input A, B, Cin);
  wire c1, s1, c2;
  my1bithalfadder h1(s1, c1, A, B);
  my1bithalfadder h2(S, c2, s1, Cin);
  my1bitmux m4(C, c1, 1'b1, c2);
endmodule

module my16bitmux(output [15:0] Out,
 input [15:0] C, D, input sel);
 // no internal nets or registers
 my1bitmux mg15 (Out[15], C[15], D[15], sel);
 my1bitmux mg14 (Out[14], C[14], D[14], sel);
 my1bitmux mg13 (Out[13], C[13], D[13], sel);
 my1bitmux mg12 (Out[12], C[12], D[12], sel);
 my1bitmux mg11 (Out[11], C[11], D[11], sel);
 my1bitmux mg10 (Out[10], C[10], D[10], sel);
 my1bitmux mg9 (Out[9], C[9], D[9], sel);
 my1bitmux mg8 (Out[8], C[8], D[8], sel);
 my1bitmux mg7 (Out[7], C[7], D[7], sel);
 my1bitmux mg6 (Out[6], C[6], D[6], sel);
 my1bitmux mg5 (Out[5], C[5], D[5], sel);
 my1bitmux mg4 (Out[4], C[4], D[4], sel);
 my1bitmux mg3 (Out[3], C[3], D[3], sel);
 my1bitmux mg2 (Out[2], C[2], D[2], sel);
 my1bitmux mg1 (Out[1], C[1], D[1], sel);
 my1bitmux mg0 (Out[0], C[0], D[0], sel);
endmodule

module my16bitfulladder(
  output wire C,
  output wire [15:0] S,
  input wire [15:0] A,
  input wire [15:0] B,
  input wire Cin
  );
  
  wire c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15;
  my1bitfulladder f1(S[0], c1, A[0], B[0], Cin);
  my1bitfulladder f2(S[1], c2, A[1], B[1], c1);
  my1bitfulladder f3(S[2], c3, A[2], B[2], c2);
  my1bitfulladder f4(S[3], c4, A[3], B[3], c3);
  my1bitfulladder f5(S[4], c5, A[4], B[4], c4);
  my1bitfulladder f6(S[5], c6, A[5], B[5], c5);
  my1bitfulladder f7(S[6], c7, A[6], B[6], c6);
  my1bitfulladder f8(S[7], c8, A[7], B[7], c7);
  my1bitfulladder f9(S[8], c9, A[8], B[8], c8);
  my1bitfulladder f10(S[9], c10, A[9], B[9], c9);
  my1bitfulladder f11(S[10], c11, A[10], B[10], c10);
  my1bitfulladder f12(S[11], c12, A[11], B[11], c11);
  my1bitfulladder f13(S[12], c13, A[12], B[12], c12);
  my1bitfulladder f14(S[13], c14, A[13], B[13], c13);
  my1bitfulladder f15(S[14], c15, A[14], B[14], c14);
  my1bitfulladder f16(S[15], C, A[15], B[15], c15);
endmodule

module my16bitaddsub_gate(output [15:0] O, output Cout, input [15:0] A, B, input S);
  wire [15:0] m;
  wire [15:0] inv;
  supply1 pwr;
  supply0 gnd;
  
  my1bitmux ms16(inv[15], pwr, gnd, B[15]);
  my1bitmux ms15(inv[14], pwr, gnd, B[14]);
  my1bitmux ms14(inv[13], pwr, gnd, B[13]);
  my1bitmux ms13(inv[12], pwr, gnd, B[12]);
  my1bitmux ms12(inv[11], pwr, gnd, B[11]);
  my1bitmux ms11(inv[10], pwr, gnd, B[10]);
  my1bitmux ms10(inv[9], pwr, gnd, B[9]);
  my1bitmux ms9(inv[8], pwr, gnd, B[8]);
  my1bitmux ms5(inv[7], pwr, gnd, B[7]);
  my1bitmux ms6(inv[6], pwr, gnd, B[6]);
  my1bitmux ms7(inv[5], pwr, gnd, B[5]);
  my1bitmux ms8(inv[4], pwr, gnd, B[4]);
  my1bitmux ms1(inv[3], pwr, gnd, B[3]);
  my1bitmux ms2(inv[2], pwr, gnd, B[2]);
  my1bitmux ms3(inv[1], pwr, gnd, B[1]);
  my1bitmux ms4(inv[0], pwr, gnd, B[0]);
  
  
  my16bitmux m4b(m, B, inv, S);
  my16bitfulladder f4b(Cout, O, A, m, S);
endmodule

/*module mult(input [15:0] y, input[15:0] x, output reg [15:0] s);
reg [15:0] p0, p1, p2, p3, p4, p5, p6, p7,p8,p9,p10,p11,p12,p13,p14,p15;
always @(*) begin
    p0 = x[0] ? y : 16'd0; 
    p1 = (x[1] ? y : 16'd0);
    p2 = (x[2] ? y : 16'd0);
    p3 = (x[3] ? y : 16'd0);
    p4 = (x[4] ? y : 16'd0);
    p5 = (x[5] ? y : 16'd0);
    p6 = (x[6] ? y : 16'd0);
    p7 = (x[7] ? y : 16'd0);
    p8 = (x[8] ? y : 16'd0);
    p9 = (x[9] ? y : 16'd0);
    p10 = (x[10] ? y : 16'd0);
    p11 = (x[11] ? y : 16'd0);
    p12 = (x[12] ? y : 16'd0);
    p13 = (x[13] ? y : 16'd0);
    p14 = (x[14] ? y : 16'd0);
    p15 = (x[15] ? y : 16'd0);

    s = p0 
        + (p1 << 4'd1) 
        + (p2 << 4'd2) 
        + (p3 << 4'd3)
        + (p4 << 4'd4)
        + (p5 << 4'd5)
        + (p6 << 4'd6)
        + (p7 << 4'd7)
        + (p8 << 4'd8) 
        + (p9 << 4'd9) 
        + (p10 << 4'd10)
        + (p11 << 4'd11)
        + (p12 << 4'd12)
        + (p13 << 4'd13)
        + (p14 << 4'd14)
        + (p15 << 4'd15);
end
endmodule
*/
module mult(input [7:0] y, input[7:0] x, output  [15:0] out);
reg [7:0] p0, p1, p2, p3, p4, p5, p6, p7;
reg [15:0] s;
always @(*) begin
    p0 = x[0] ? y : 8'd0; 
    p1 = (x[1] ? y : 8'd0);
    p2 = (x[2] ? y : 8'd0);
    p3 = (x[3] ? y : 8'd0);
    p4 = (x[4] ? y : 8'd0);
    p5 = (x[5] ? y : 8'd0);
    p6 = (x[6] ? y : 8'd0);
    p7 = (x[7] ? y : 8'd0);

    s = p0 
        + (p1 << 3'd1) 
        + (p2 << 3'd2) 
        + (p3 << 3'd3)
        + (p4 << 3'd4)
        + (p5 << 3'd5)
        + (p6 << 3'd6)
        + (p7 << 3'd7);
end
assign out = s;
endmodule

module muxxor(output y, input a, input b);
  supply0 gnd;
  supply1 pwr;
  wire between;
  my1bitmux mux2(between,pwr,gnd,b);
  my1bitmux mux3(y,b,between,a);
endmodule

module muxnot(output y, input a);
  supply0 gnd;
  supply1 pwr;
  my1bitmux mux5(y,pwr,gnd,a);
endmodule

module my16bitadd(output [15:0] O, output Cout, input [15:0] A, B, input S);  
  supply0 gnd;
  wire [15:0] b_n,b_carry;
  muxnot n0(b_n[0], B[0]);
  muxnot n1(b_n[1], B[1]);
  muxnot n2(b_n[2], B[2]);
  muxnot n3(b_n[3], B[3]);
  muxnot n4(b_n[4], B[4]);
  muxnot n5(b_n[5], B[5]);
  muxnot n6(b_n[6], B[6]);
  muxnot n7(b_n[7], B[7]);
  muxnot n8(b_n[8], B[8]);
  muxnot n9(b_n[9], B[9]);
  muxnot n10(b_n[10], B[10]);
  muxnot n11(b_n[11], B[11]);
  muxnot n12(b_n[12], B[12]);
  muxnot n13(b_n[13], B[13]);
  muxnot n14(b_n[14], B[14]);
  muxnot n15(b_n[15], B[15]);
  my16bitmux m0(b_carry,B,b_n,S);
  my16bitfulladder fa0(Cout,O,A,b_carry,S);
endmodule

module my16bitxor(output y, input a, input b);
output [15:0] y;
input [15:0] a;
input [15:0] b;

muxxor x0(y[0], a[0], b[0]);
muxxor x1(y[1], a[1], b[1]);
muxxor x2(y[2], a[2], b[2]);
muxxor x3(y[3], a[3], b[3]);
muxxor x4(y[4], a[4], b[4]);
muxxor x5(y[5], a[5], b[5]);
muxxor x6(y[6], a[6], b[6]);
muxxor x7(y[7], a[7], b[7]);
muxxor x8(y[8], a[8], b[8]);
muxxor x9(y[9], a[9], b[9]);
muxxor x10(y[10], a[10], b[10]);
muxxor x11(y[11], a[11], b[11]);
muxxor x12(y[12], a[12], b[12]);
muxxor x13(y[13], a[13], b[13]);
muxxor x14(y[14], a[14], b[14]); 
muxxor x15(y[15], a[15], b[15]);
endmodule


module ram(addr,we,d,q);
input [7:0] addr;
input we;
input [15:0] d;
output reg [15:0] q;
reg [15:0] MEM [0:255]; 
always @(*)
begin
  if(we)
MEM[addr] <= d;
else
  q <= MEM[addr];
end
endmodule

module alu(A,B,opAlu,Rout);
input [15:0] A;
input [15:0] B;
input [1:0] opAlu;
output reg [15:0] Rout;


reg [2:0] state, next;
wire [15:0] xor_out;
wire [15:0] add_out;
wire [15:0] sub_out;
supply0 gnd;
supply1 pwr;
wire co;
wire cm;
wire [15:0] mult_out;

my16bitxor x0(xor_out, A, B); // xor
my16bitaddsub_gate a0(add_out, co, A, B, gnd); // add
my16bitaddsub_gate s0(sub_out, cm, B, A, pwr); // sub
mult m0(A,B,mult_out); // multiply

parameter S0 = 2'b00,  
          S1 = 2'b01,  
          S2 = 2'b10,  
          S3 = 2'b11;

always @ (*)
case(opAlu)

   S0: begin
   Rout <= xor_out;
   end
   
   S1: begin
   Rout <= add_out;
   end
   
   S2: begin
   Rout <= mult_out;
   end
   
   S3: begin
   Rout <= sub_out;
   end
   
   default: begin
   Rout <= 16'b0000_0000_0000_0000;
   end
   endcase
 
endmodule

module ctr (
clk,
rst,
A,
B,
zflag,
opcode,
Q,
Done,
muxPC,
muxMAR,
muxACC,
loadMAR,
loadPC,
loadACC,
loadMDR,
loadIR,
opALU,
MemRW
);

input clk;
input rst;
input zflag;
input [7:0]opcode;
input [15:0] A, B;
output reg [15:0] Q;
output reg Done;
output reg muxPC;
output reg muxMAR;
output reg muxACC;
output reg loadMAR;
output reg loadPC;
output reg loadACC;
output reg loadMDR;
output reg loadIR;
output reg [1:0] opALU;
output reg MemRW;

reg [15:0] R;
reg [7:0] I;
reg [7:0] R_temp;
wire C_out;
reg S;
reg load;
reg [15:0] Q_temp;

reg [15:0] A_reg, B_reg, A_temp, B_temp;
wire [15:0] O_temp;
reg unused;

reg [4:0] state, next;

parameter Fetch_1 = 5'b00000,
          Fetch_2 = 5'b00001,
          Fetch_3 = 5'b00010,
          Decode = 5'b00011,
          ExecADD_1 = 5'b00100,
          ExecADD_2 = 5'b00101,
          ExecXOR_1 = 5'b00110,
          ExecXOR_2 = 5'b00111,
          ExecLoad_1 = 5'b01000,
          ExecLoad_2 = 5'b01001,
          ExecStore = 5'b01010,
          ExecJump = 5'b01011,
          ExecSUB_1 = 5'b01100,
          ExecSUB_2 = 5'b01101,
          ExecMUL_1 = 5'b01110,
          ExecMUL_2 = 5'b01111,
          ExecDIV_1 = 5'b10000,
          ExecDIV_2 = 5'b10001,
          ExecDIV_3 = 5'b10010,
          ExecDIV_4 = 5'b10011,
          ExecDIV_5 = 5'b10100,
          ExecDIV_6 = 5'b10101,
          ExecDIV_7 = 5'b10110;
          
          
always @ (posedge clk)
  begin
    if (rst)
      state <= Fetch_1;
    else
      state <= next;
  end
  
//alu div(A_temp, B_temp, opALU, O_temp);
my16bitaddsub_gate a1(O_temp, C_out, A_temp, B_temp, S);  
always @ (*)
  
    case(state)
      
      Fetch_1: begin
        I = 0;
        MemRW = 0;
        muxMAR = 0;
        muxPC = 0;
        loadPC = 1;
        loadMAR = 1;
        loadIR = 0;
        loadMDR = 0;
        loadACC = 0;
        load = 0;
        Done = 0;
        next <= Fetch_2;
      end
      
      Fetch_2: begin
        loadMDR = 1;
        loadPC = 0;
        loadMAR = 0;
        next <= Fetch_3;
      end
      
      Fetch_3: begin
        loadIR = 1;
        loadMDR = 0;
        next <= Decode;
      end
      
      Decode: begin
          loadIR = 0;
          muxMAR = 1;
          loadMAR = 1;
         
           if(opcode == 8'h01) next <= ExecADD_1;
          
            else if(opcode == 8'h02) next <= ExecSUB_1;
            
              else if(opcode == 8'h03) next <= ExecMUL_1;
              
                else if(opcode == 8'h04) 
                  begin
                    load = 1;
                    next <= ExecDIV_1;
                  end
                
                  else if(opcode == 8'h05) next <= ExecXOR_1;
                  
                    else if(opcode == 8'h06) next <= ExecJump;
                    
                      else if((opcode == 8'h07) && (zflag)) next <= ExecJump;
                      
                        else if((opcode == 8'h07) && (!zflag)) next <= Fetch_1;
                        
                          else if(opcode == 8'h08) next <= ExecStore;
                          
                            else if(opcode == 8'h09) next <= ExecLoad_1;
                            
                              else next <= Fetch_1;
                                
                              
      end 
      
      ExecADD_1: begin
        loadMAR = 0;
        MemRW = 0;
        loadMDR = 1;
        next <= ExecADD_2;
      end
      
      ExecADD_2: begin
        loadMDR = 0;
        loadACC = 1;
        muxACC = 0;
        opALU = 1;
        next <= Fetch_1;
      end
      
      ExecSUB_1: begin
        loadMAR = 0;
        MemRW = 0;
        loadMDR = 1;
        next <= ExecSUB_2;
      end
      
      ExecSUB_2: begin
        loadMDR = 0;
        loadACC = 1;
        muxACC = 0;
        opALU = 3;
        next <= Fetch_1;
      end
      
      ExecMUL_1: begin
        loadMAR = 0;
        MemRW = 0;
        loadMDR = 1;
        next <= ExecMUL_2;
      end
      
      ExecMUL_2: begin
        loadMDR = 0;
        loadACC = 1;
        muxACC = 0;
        opALU = 2;
        next <= Fetch_1;
      end
      
      ExecDIV_1: begin
        loadMAR = 0;
        MemRW = 0;
        loadMDR = 1;
        if(load)
          begin
            Q_temp = 0;
            Q = 0;
            Done = 0;
            next = ExecDIV_2;
          end
        else
          next = ExecDIV_1;
        end
      
      ExecDIV_2: begin
        
            A_reg = A;
            B_reg = B;
            R = A_reg;
            R_temp = A_reg;
            next = ExecDIV_3;
          end
          
      ExecDIV_3 : begin
        if(R >= B_reg)
          begin 
          I = I+1;
          //S=1;
          next = ExecDIV_4;
        end
      else 
        next = ExecDIV_7; 
      end
      
      // select 1 for subtract
      
      ExecDIV_4: begin
        A_temp = R;
        B_temp = B_reg;
        S = 1; 
       // opALU = 3; // selects SUB
       // R = O_temp; 
        //R_temp = O_temp;
        R_temp <= O_temp;
        next = ExecDIV_5;
      end
      
      // select 0 for add
      
      ExecDIV_5: begin
       R <= R_temp;
        unused = C_out;
        A_temp = Q;
        B_temp = 1;
        S = 0; 
        //opALU = 1; // selects ADD
        next = ExecDIV_6;
      end
      
      ExecDIV_6: begin
       
        Q = O_temp;
        unused = C_out;
        next = ExecDIV_3;
      end
      
      ExecDIV_7: begin
        loadMDR = 0;
        loadACC = 1;
        muxACC = 0;
        Done = 1;
        next = Fetch_1;
      end
      
      
      ExecXOR_1: begin
        loadMAR = 0;
        MemRW = 0;
        loadMDR = 1;
        next <= ExecXOR_2;
      end
      
      ExecXOR_2: begin
        loadMDR = 0;
        loadACC = 1;
        muxACC = 0;
        opALU = 0;
        next <= Fetch_1;
      end
      
      ExecJump : begin
        loadMAR = 0;
        muxPC = 1;
        loadPC = 1;
        next <= Fetch_1;
      end
      
      ExecStore: begin
        loadMAR = 0;
        MemRW = 1;
        next <= Fetch_1;
      end
      
      ExecLoad_1: begin
        loadMAR = 0;
        MemRW = 0;
        loadMDR = 1;
        next <= ExecLoad_2;
      end
      
      ExecLoad_2: begin
        loadMDR = 0;
        loadACC = 1;
        muxACC = 1;
        next <= Fetch_1;
      end
      
      default: begin
        A_temp = 0;
        B_temp = 0;
        S = 0;
        A_reg = 0;
        B_reg = 0;
        R = 0;
        Done = 0;
        unused = 0;
        load = 0;
        next <= Fetch_1;
      end
    endcase
endmodule

module registers(
clk,
rst,
loadMAR,
loadPC,
loadACC,
loadMDR,
loadIR,
PC_reg,
PC_next,
IR_reg,
IR_next,
ACC_reg,
ACC_next,
MDR_reg,
MDR_next,
MAR_reg,
MAR_next,
Zflag_reg,
zflag_next
);

parameter S0 = 8'b0000_0000,  
          S1 = 16'b0000_0000_0000_0000;

input wire clk;
input wire rst;
output reg [7:0]PC_reg;
input wire [7:0]PC_next;
output reg [15:0]IR_reg;
input wire [15:0]IR_next;
output reg [15:0]ACC_reg;
input wire [15:0]ACC_next;
output reg [15:0]MDR_reg;
input wire [15:0]MDR_next;
output reg [7:0]MAR_reg;
input wire [7:0]MAR_next;
output reg Zflag_reg;
input wire zflag_next;
input loadMAR;
input loadPC;
input loadACC;
input loadMDR;
input loadIR;

always @(posedge clk)
begin
if(rst)
begin
PC_reg = 0;
IR_reg = 0;
ACC_reg = 0;
MDR_reg = 0;
MAR_reg = 0;
Zflag_reg = 0;
end
else
  begin
/*
else
  begin

 //if(loadPC)
    PC_reg <= PC_next;

  //  if(loadIR)
        IR_reg <= IR_next;
    
    //  if(loadACC)
          ACC_reg <= ACC_next;
          
      //  if(loadMDR)
          MDR_reg <= MDR_next;
      
        //  if(loadMAR)
            MAR_reg <= MAR_next;
            
            Zflag_reg <= zflag_next;
            end
          
end
*/

//always @(*)
//begin
  if(loadPC)
      PC_reg <= PC_next;
   //else
     // PC_next <= PC_reg;
  //  end

//always @ (*)
//begin
  if(loadIR)
      IR_reg <= IR_next;
    //else 
    //IR_next <= IR_reg;
    //end
    
    //always @ (*)
    //begin
      if(loadACC)
          ACC_reg <= ACC_next;
        //else
          //ACC_next <= ACC_reg;
     //   end
      
      //always @ (*)
      //begin
        if(loadMDR)
        MDR_reg <= MDR_next;
      //else
          //MDR_reg = 0;
      //end
      
      //always @ (*)
      //begin
        if(loadMAR)
            MAR_reg <= MAR_next;
           //else
            //MAR_reg = 0;
            Zflag_reg <= zflag_next;
          end
          end
endmodule

module datapath(
clk,
rst,
Done,
Q,
ACC_reg,
MDR_reg,
muxPC,
muxMAR,
muxACC,
loadMAR,
loadPC,
loadACC,
loadMDR,
loadIR,
opALU,
zflag,
opcode,
MemAddr,
MemD,
MemQ
);
input clk;
input rst;
input Done;
input muxPC;
input muxMAR;
input muxACC;
input loadMAR;
input loadPC;
input loadACC;
input loadMDR;
input loadIR;
input [1:0] opALU;

output reg zflag;
output reg [7:0]opcode;
output reg [7:0]MemAddr;
output reg [15:0]MemD;
input [15:0]MemQ;
output [15:0] Q;

reg [7:0]PC_next;
reg [15:0]IR_next;
reg [15:0]ACC_next;
reg [15:0]MDR_next;
reg [7:0]MAR_next;
reg zflag_next;

wire [7:0]PC_reg;
wire [15:0]IR_reg;
input wire [15:0]ACC_reg;
input wire [15:0]MDR_reg;
wire [7:0]MAR_reg;
wire zflag_reg;
wire [15:0] ALU_out;

           registers r0(
clk,
rst,
loadMAR,
loadPC,
loadACC,
loadMDR,
loadIR,
PC_reg,
PC_next,
IR_reg,
IR_next,
ACC_reg,
ACC_next,
MDR_reg,
MDR_next,
MAR_reg,
MAR_next,
zflag_reg,
zflag_next
);

alu d0(MDR_reg, ACC_reg, opALU, ALU_out);


always @(*)
begin
  //if(loadPC)
      PC_next <= muxPC ? IR_reg [15:8] : PC_reg+1;
    //else
      //PC_next <= PC_reg;
    end

always @ (*)
begin
  //if(loadIR)
      IR_next <= MDR_reg;
    //else 
    //IR_next <= IR_reg;
    end
    
    always @ (*)
    begin
     // if(loadACC)
     if(Done)
       ACC_next <= muxACC ? MDR_reg : Q;
     else
          ACC_next <= muxACC ? MDR_reg : ALU_out;
       // else
         // ACC_next <= ACC_reg;
        end
      
      always @ (*)
      begin
        //if(loadMDR)
        MDR_next <= MemQ;
      //else
        //MDR_next <= MDR_reg;
      end
      
      always @ (*)
      begin
        //if(loadMAR)
            MAR_next <= muxMAR ? IR_reg [15:8] : PC_reg;
          //else
            //MAR_next <= MAR_reg;
          end
        
        always @ (*)
        begin
          if(ACC_reg == 0)
            zflag_next = 1;
          else
            zflag_next = 0;
          end
          
          always @ (*)
          begin
            if(ACC_reg == 0)
              zflag = 1;
            else
              zflag = 0;
            end
            
            always @ (*)
            begin
              opcode <= IR_reg [7:0];
            end
            
            always @ (*)
            begin
              MemAddr <= MAR_reg;
            end
            
            always @ (*)
            begin
              MemD <= ACC_reg;
            end
            

endmodule

module proj1(
clk,
rst,
MemRW_IO,
MemAddr_IO,
MemD_IO
);

input clk;
input rst;
output MemRW_IO;
output [7:0]MemAddr_IO;
output [15:0]MemD_IO;

wire MemRW;
wire [7:0] MemAddr;
wire [15:0] MemD;
wire [15:0] MemQ;

wire zflag;
wire [7:0] opcode;
wire [15:0] Q;
wire Done;
wire muxPC;

wire muxMAR;
wire muxACC;
wire loadMAR;
wire loadPC;
wire loadACC;

wire loadMDR;
wire loadIR;
wire [1:0] opALU;
wire[7:0]PC_next;
wire [15:0]IR_next;
wire [15:0]ACC_next;
wire [15:0]MDR_next;
wire [7:0]MAR_next;
wire zflag_next;

wire [7:0]PC_reg;
wire [15:0]IR_reg;
wire [15:0]ACC_reg;
wire [15:0]MDR_reg;
wire [7:0]MAR_reg;
wire [15:0] ALU_out;


ram k0(MemAddr, MemRW, MemD, MemQ);
ctr c0(clk, rst, ACC_reg, MDR_reg, zflag, opcode, Q, Done, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, MemRW);
datapath data(clk, rst, Done, Q, ACC_reg, MDR_reg, muxPC, muxMAR, muxACC, loadMAR, loadPC, loadACC, loadMDR, loadIR, opALU, zflag, opcode, MemAddr, MemD, MemQ);


assign MemAddr_IO = MemAddr;
assign MemD_IO = MemD;
assign MemRW_IO = MemRW;

endmodule

module proj1_tb();
  
reg clk;
reg rst;
wire MemRW_IO;
wire [7:0]MemAddr_IO;
wire [15:0]MemD_IO;

proj1 dut(clk, rst, MemRW_IO, MemAddr_IO, MemD_IO);

always
#5 clk = !clk;
initial begin
clk=1'b0;
rst=1'b1;
$readmemh("memory.list", proj1_tb.dut.k0.MEM);
#20 rst=1'b0;
#40000 //might need to be very large
$display("Final value\n");
$display("0x000e %d\n",proj1_tb.dut.k0.MEM[16'h000e]);
$finish;
end
endmodule


module alu_tb();
reg [7:0] A;
reg [7:0] B;
wire [15:0] Rout;
reg [1:0] opAlu;
alu dut(.A(A), .B(B), .opAlu(opAlu), .Rout(Rout));
initial begin
opAlu = 2'd2;
A = 8'd3;
B = 8'd2;
$monitor($time, " opAlu=%d,Rout=%b ", opAlu, Rout);

end
endmodule

module ctr_tb();
  
  reg load, clk, rst;
  reg [7:0] A, B;
  reg zflag;
  reg [7:0] opcode;
  
  wire [7:0] Q;
  wire Done;
  wire muxPC;
  wire muxMAR;
  wire muxACC;
  wire loadMAR;
  wire loadPC;
  wire loadACC;
  wire loadMDR;
  wire loadIR;
  wire opALU;
  wire MemRW;
  
ctr dut(
clk,
rst,
load,
A,
B,
zflag,
opcode,
Q,
Done,
muxPC,
muxMAR,
muxACC,
loadMAR,
loadPC,
loadACC,
loadMDR,
loadIR,
opALU,
MemRW
);
  
  always #5 clk = ~clk;
  
  
  initial begin
    
  zflag = 0;
  opcode = 8'h04;
  
  clk = 1'b0; rst = 1'b0; load = 1'b1;
  #10 A = 8'd35; B = 8'd7; 
  @(posedge Done) $display ("A = %d, B = %d, Q = %d",A,B,Q);
  #10 $finish;
end
endmodule
