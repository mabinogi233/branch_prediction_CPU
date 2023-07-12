`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2020/05/27 19:36:56
// Design Name: 
// Module Name: myCPU
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////





module MainDecoder(
	input [31:0] instr,
	output reg regwrite,
	output reg  regdst,
	output reg  alusrc,
	output reg branch,
	output reg memWrite,
	output reg memtoReg,
	output reg [1:0] aluop,
	output reg jump
);
    wire [5:0] op;
    assign op = instr[31:26];
    always @(*)begin
        
        case(op)
            6'b000000:begin
				regwrite<=1'b1;
				regdst<=1'b1;
				alusrc<=1'b0;
				branch<=1'b0;
				memWrite<=1'b0;
				memtoReg<=1'b0;
				aluop<=2'b10;
				jump<=1'b0;	
			end	
			6'b100011:begin
				regwrite<=1'b1;
				regdst<=1'b0;
				alusrc<=1'b1;
				branch<=1'b0;
				memWrite<=1'b0;
				memtoReg<=1'b1;
				aluop<=2'b00;
				jump<=1'b0;		
			end				
			6'b101011:begin
				regwrite<=1'b0;
				regdst<=1'b0;
				alusrc<=1'b1;
				branch<=1'b0;
				memWrite<=1'b1;
				memtoReg<=1'b0;
				aluop<=2'b00;
				jump<=1'b0;		
			end			
			6'b000100:begin
				regwrite<=1'b0;
				regdst<=1'b0;
				alusrc<=1'b0;
				branch<=1'b1;
				memWrite<=1'b0;
				memtoReg<=1'b0;
				aluop<=2'b01;
				jump<=1'b0;	
			end				
			6'b001000:begin
				regwrite<=1'b1;
				regdst<=1'b0;
				alusrc<=1'b1;
				branch<=1'b0;
				memWrite<=1'b0;
				memtoReg<=1'b0;
				aluop<=2'b00;
				jump<=1'b0;	
			end					
			6'b000010:begin
				regwrite<=1'b0;
				regdst<=1'b0;
				alusrc<=1'b0;
				branch<=1'b0;
				memWrite<=1'b0;
				memtoReg<=1'b0;
				aluop<=2'b00;
				jump<=1'b1;	
			end	
			default:begin
			    regwrite<=1'b0;
				regdst<=1'b0;
				alusrc<=1'b0;
				branch<=1'b0;
				memWrite<=1'b0;
				memtoReg<=1'b0;
				aluop<=2'b00;
				jump<=1'b0;
			end	
		endcase
		
	end
endmodule


module aluController(
	input [1:0] aluop,
	input [5:0] op,
	output reg [2:0] alucontroll
);
	always @(*)begin
		casex({aluop,op})
			8'b10100000: alucontroll<=3'b010;
			8'b10100010:alucontroll<=3'b110;
			8'b10100100:alucontroll<=3'b000;
			8'b10100101:alucontroll<=3'b001;
			8'b10101010:alucontroll<=3'b111;
			8'b00xxxxxx:alucontroll<=3'b010;
			8'b01xxxxxx:alucontroll<=3'b110;
		endcase;
	end
endmodule

//����������
module Controller(
	input [31:0] instr,
	input zero,
	output memtoReg,
	output memWrite,
	output pcsrc,
	output alusrc,
	output regdst,
	output regwrite,
	output jump,
	output [2:0] alucontroll,
	output branch
);
	wire [1:0] aluop;
	//wire branch;
	MainDecoder mydecoder(instr,regwrite,regdst,alusrc,branch,memWrite,memtoReg,aluop,jump);
	aluController myalucontroller(aluop,instr[5:0],alucontroll);
	assign pcsrc = branch & zero;
	
endmodule 




module PCmodule(
	input clk,
	input rst,
	input en,
	input [31:0] pcin,
	output reg [31:0] pc
);
	always @(posedge clk,posedge rst)begin
		if(rst)begin
			pc<=32'b00000000000000000000000000000000;
		end else if(en) begin
			pc<=pcin;
		end
	end
endmodule

//�ӷ���
module addmodule(
	input [31:0] pcin,
	input [31:0] jumpnum,
	output [31:0] pcout
);
	assign pcout = pcin + jumpnum;
endmodule


//ALUģ��
module Myalu(
    input [31:0] A,
    input [31:0] B,
    input [2:0] op,
    output [31:0] C
    );
    reg [31:0] x;
        always @(*) begin
            case(op)
                3'b000:x<=A&B;
                3'b001:x<=A|B;
                3'b010:x<=A+B; 
                3'b011:;
                3'b100:x<=~A;
                3'b101:;
                3'b110:x<=A-B;
                3'b111:x<=(A<B?32'b00000000000000000000000000000001:32'b00000000000000000000000000000000);
                default:;
            endcase
        end 
        assign C = x;
endmodule

//�з�����չ16-32
module signext(
	input wire[15:0] a,
	output wire[31:0] y
    );
	assign y = {{16{a[15]}},a};
endmodule

//����λģ��
module sl2(
    input wire[31:0] a,
	output wire[31:0] y
    );
	assign y = {a[29:0],2'b00};
endmodule


//�Ĵ�����
module regfile(
	input wire clk,
	input wire we3,
	input wire[4:0] ra1,ra2,wa3,
	input wire[31:0] wd3,
	output wire[31:0] rd1,rd2
    );
	reg [31:0] rf[31:0];
	always @(negedge clk) begin
		if(we3) begin
			 rf[wa3] <= wd3;
		end
	end
	assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
	assign rd2 = (ra2 != 0) ? rf[ra2] : 0;
endmodule

//��ѡһѡ����
module mux2 #(parameter WIDTH = 8)(
	input wire[WIDTH-1:0] x1,x2,
	input wire a,
	output wire[WIDTH-1:0] y
    );
	assign y = a ? x2 : x1;
endmodule


//��ѡһѡ����
module mux3 #(parameter WIDTH = 8)(
	input wire[WIDTH-1:0] x1,x2,x3,
	input wire [1:0] a,
	output reg [WIDTH-1:0] y
    );
	always @(*) begin
	   case(a)
	       2'b00:y<=x1;
	       2'b01:y<=x2;
	       2'b10:y<=x3;
	   endcase    
	end
endmodule


//�Ĵ���
module flopr #(parameter WIDTH = 8)(
	input wire clk,rst,
	input wire [WIDTH-1:0] in,
	output reg [WIDTH-1:0] out
    );
	always @(posedge clk,posedge rst) begin
	   if(rst)begin
	       out<=0; 
	   end else begin
	       out<=in;
	   end
	end
endmodule

module floprc #(parameter WIDTH = 8)(
	input wire clk,rst,clear,
	input wire [WIDTH-1:0] in,
	output reg [WIDTH-1:0] out
    );
	always @(posedge clk,posedge rst) begin
	   if(rst)begin
	       out<=0; 
	   end else if(clear) begin
	       out<=0; 
	   end else begin
	       out<=in;
	   end
	end
endmodule

module flopenrc #(parameter WIDTH = 8)(
	input wire clk,rst,en,clear,
	input wire [WIDTH-1:0] in,
	output reg [WIDTH-1:0] out
    );
	always @(posedge clk,posedge rst) begin
	   if(rst)begin
	       out<=0; 
	   end else if(clear) begin
	       out<=0; 
	   end else if(en) begin
	       out<=in;
	   end
	end
endmodule

module flopenr #(parameter WIDTH = 8)(
	input wire clk,rst,en,
	input wire [WIDTH-1:0] in,
	output reg [WIDTH-1:0] out
    );
	always @(posedge clk,posedge rst) begin
	   if(rst)begin
	       out<=0; 
	   end else if(en) begin
	       out<=in;
	   end
	end
endmodule


module mips(
	input wire clk,rst,
	output wire[31:0] pcF,
	input wire[31:0] instrF,
	output wire memwriteM,
	output wire[31:0] aluout,writedata,
	input wire[31:0] readdata
    );

	wire memtoregD,alusrcD,regdstD,regwriteD,pcsrcD,zeroD,memwriteD;
	wire[2:0] alucontrolD;
	wire [31:0] instrD;
	wire jumpD;
	wire branchD;
    //������


	Controller c(instrD,zeroD,memtoregD,
		memwriteD,pcsrcD,alusrcD,regdstD,regwriteD,jumpD,alucontrolD,branchD);
    //������·
	datapath dp(    
	.clk(clk),
	.rst(rst),
    .instrF(instrF),
    .instrD(instrD),
    .jumpD(jumpD),
    .regwriteD(regwriteD),
    .memtoregD(memtoregD),
    .memwriteD(memwriteD),
    .alucontrollD(alucontrolD),
    .alusrcD(alusrcD),
    .regdstD(regdstD),
    .pcsrcD(pcsrcD),
    .zero(zeroD),
    .pcF(pcF),
    .aluoutM(aluout),
    .writedataM(writedata),
    .readdataM(readdata),
    .branchD(branchD),
    .memwriteM(memwriteM)
    );
endmodule

//����
module top(
	input wire clk,rst,
	output wire[31:0] writedata,dataadr,
	output wire memwrite,
	output [31:0] instr
    );
	wire[31:0] pc,readdata;
	//CPU����
	mips mips(clk,rst,pc,instr,memwrite,dataadr,writedata,readdata);
	//�洢��
	inst_mem inst_ram (
        .clka(~clk),    // input wire clka
        .ena(1'b1),      // input wire ena
        .wea(4'b0000),      // input wire [3 : 0] wea
        .addra(pc[9:2]),  // input wire [7 : 0] addra
        .dina(32'b0),    // input wire [31 : 0] dina
        .douta(instr)  // output wire [31 : 0] douta
    );

	data_mem data_ram (
        .clka(~clk),    // input wire clka
        .ena(1'b1),      // input wire ena
        .wea({4{memwrite}}),      // input wire [3 : 0] wea
        .addra(dataadr[9:0]),  // input wire [9 : 0] addra
        .dina(writedata),    // input wire [31 : 0] dina
        .douta(readdata)  // output wire [31 : 0] douta
);
endmodule







//������·
module datapath(
    input wire clk,rst,
    input wire [31:0] instrF,
    output wire [31:0] instrD,
    input wire jumpD,
    input wire regwriteD,memtoregD,memwriteD,
    input wire [2:0] alucontrollD,
    input wire alusrcD,regdstD,
    input wire branchD,
    input wire pcsrcD,
    output wire zero,
    output wire[31:0] pcF,
    output wire [31:0] aluoutM,writedataM,
    input wire [31:0] readdataM,
    output wire memwriteM
    );
    //��һ��
    wire [31:0] pcpie;
    wire startIF; //����ð��ģ��
    wire [31:0] pcplus4F;
    wire [31:0] pcbranchD;
    wire startID;//����ð��ģ��
    wire [31:0] pcplus4D;
    wire [31:0] resultW;
    wire regwriteW;//���弶
    wire [4:0] writeregW;//���弶
    wire [31:0] rd1D;
    wire [31:0] rd2D;
    wire [31:0] newrd1D,newrd2D;
    wire [31:0] signlmmD;
    wire [31:0] signlmmD2;
    wire startIE;
    wire [4:0] rsE,rtE,rdE;
    wire regwriteE,memtoregE,memwriteE,alusrcE,regdstE;
    wire [2:0] alucontrollE;
    wire [31:0] rd1E;
    wire [31:0] rd2E;
    wire [31:0] signlmmE;
    wire [31:0] srcAE;
    wire [1:0] forwardAE;
    wire [1:0] forwardBE;
    wire [31:0] writedataE;
    wire regwriteM,memtoregM;
    wire [4:0] writeregE,writeregM;
    wire [31:0] aluoutE;
    wire [31:0] srcBE;
    wire [31:0] readdataW,aluoutW;
    wire forwardAD,forwardBD;
    wire memtoregW;
    wire [31:0] pcnextbrFD;
    
    //��̬��֧Ԥ�ⲹ��ı���
    wire pcsrcFyuce,jumpFyuce;
    wire pcsrcDyuce,jumpDyuce;
    wire [2:0] jiucuoCode;//��λ������
    
    PCmodule mypcmodule(.clk(clk),.rst(rst),.en(~startIF),.pcin(pcpie),.pc(pcF));
    
    mux2 #(32) numux1(.x1(pcplus4F),.x2(pcbranchD),.a(pcsrcDyuce),.y(pcnextbrFD));
    
    wire[31:0] pcpieyuce;
    mux2 #(32) numux2(pcnextbrFD,{pcplus4D[31:28],instrD[25:0],2'b00},jumpDyuce,pcpieyuce);
    
    //������·
    wire[31:0] pcnextbrtrue,pcpietrue;
    mux2 #(32) numux100(.x1(pcplus4F),.x2(pcbranchD),.a(jiucuoCode[0:0]),.y(pcnextbrtrue));
    mux2 #(32) numux101(pcnextbrtrue,{pcplus4D[31:28],instrD[25:0],2'b00},jiucuoCode[1:1],pcpietrue);
    
    //�жϾ����벻����
     mux2 #(32) numux102(pcpieyuce,pcpietrue,jiucuoCode[2:2],pcpie);
    
     wire [31:0] pcE,pcD;
    //����Ԥ��ģ��
    control_dongtai mycontrol_dongtai(
        .clk(clk),
        .pc(pcF),
        .instrF(instrF),
        .pcsrcFyuce(pcsrcFyuce),
        .jumpFyuce(jumpFyuce),
        .pcsrcD(pcsrcD),
        .pcsrcDyuce(pcsrcDyuce),
        .jumpD(jumpD),
        .jumpDyuce(jumpDyuce),
        .jiucuo(jiucuoCode),
        .pcE(pcE)
    );
    
    
    addmodule myadd1(.pcin(pcF),.jumpnum(32'h4),.pcout(pcplus4F));
    
    //һ��������ź�  (not jiucuoCode[2:2]) and pcsrcDyuce or iucuoCode[0:0] �߼��Ƶ��ó�
    wire jiucuo_clear;
    assign jiucuo_clear = ((~jiucuoCode[2:2]) & pcsrcDyuce) | jiucuoCode[0:0];
    
    //��һ����

    flopenrc #(32) flopenrc_1(clk,rst,~startID,jiucuo_clear,pcplus4F,pcplus4D);
	flopenrc #(32) flopenrc_2(clk,rst,~startID,jiucuo_clear,instrF,instrD);
    
    //������
    flopenrc #(1) flopenrc_100(clk,rst,~startID,jiucuo_clear,pcsrcFyuce,pcsrcDyuce);
    flopenrc #(1) flopenrc_101(clk,rst,~startID,jiucuo_clear,jumpFyuce,jumpDyuce);
    flopenrc #(32) flopenrc_102(clk,rst,~startID,jiucuo_clear,pcF,pcD);
    
    regfile myregfile(.clk(clk),.we3(regwriteW),.ra1(instrD[25:21]),.ra2(instrD[20:16]),.wa3(writeregW),.wd3(resultW),.rd1(rd1D),.rd2(rd2D));
    
    signext mysignnext(.a(instrD[15:0]),.y(signlmmD));

    sl2 mysl2_1(.a(signlmmD),.y(signlmmD2));
    
    addmodule myadd2(.pcin(signlmmD2),.jumpnum(pcplus4D),.pcout(pcbranchD));
    
    mux2 #(32) numux3(rd1D,aluoutM,forwardAD,newrd1D);

	mux2 #(32) numux4(rd2D,aluoutM,forwardBD,newrd2D);
    
    assign zero = (newrd1D==newrd2D)?1'b1:1'b0;
    


   
    floprc #(32) flopenrc_300(clk,rst,1'b0,pcD,pcE);
    //�ڶ�����
    floprc #(1) flopenrc_5(clk,rst,startIE,regwriteD,regwriteE);
    floprc #(1) flopenrc_6(clk,rst,startIE,memtoregD,memtoregE);
    floprc #(1) flopenrc_7(clk,rst,startIE,memwriteD,memwriteE);
    floprc #(3) flopenrc_8(clk,rst,startIE,alucontrollD,alucontrollE);
    floprc #(1) flopenrc_9(clk,rst,startIE,alusrcD,alusrcE);
    floprc #(1) flopenrc_10(clk,rst,startIE,regdstD,regdstE);
    floprc #(32) flopenrc_11(clk,rst,startIE,rd1D,rd1E);
    floprc #(32) flopenrc_13(clk,rst,startIE,rd2D,rd2E);
    floprc #(5) flopenrc_14(clk,rst,startIE,instrD[25:21],rsE);
    floprc #(5) flopenrc_15(clk,rst,startIE,instrD[20:16],rtE);
    floprc #(5) flopenrc_16(clk,rst,startIE,instrD[15:11],rdE);
    floprc #(32) flopenrc_17(clk,rst,startIE,signlmmD,signlmmE);
    
    //������
    mux3 #(32) mymux3_1(.x1(rd1E),.x2(resultW),.x3(aluoutM),.a(forwardAE),.y(srcAE));
    
    mux3 #(32) mymux3_2(.x1(rd2E),.x2(resultW),.x3(aluoutM),.a(forwardBE),.y(writedataE));   
   
    mux2 #(32) mumux2(.x1(writedataE),.x2(signlmmE),.a(alusrcE),.y(srcBE));
    
    Myalu myalu(.A(srcAE),.B(srcBE),.op(alucontrollE),.C(aluoutE));
    
    mux2 #(5) mumux3(.x1(rtE),.x2(rdE),.a(regdstE),.y(writeregE));
   
    //��������
    flopr #(1) flopenrc_18(clk,rst,regwriteE,regwriteM);
    flopr #(1) flopenrc_19(clk,rst,memtoregE,memtoregM);
    flopr #(1) flopenrc_20(clk,rst,memwriteE,memwriteM);
    flopr #(32) flopenrc_21(clk,rst,aluoutE,aluoutM);
    flopr #(32) flopenrc_22(clk,rst,writedataE,writedataM);
    flopr #(5) flopenrc_23(clk,rst,writeregE,writeregM);
    
    //���ļ���
    flopr #(1) flopenrc_24(clk,rst,regwriteM,regwriteW);
    flopr #(1) flopenrc_25(clk,rst,memtoregM,memtoregW);
    flopr #(32) flopenrc_26(clk,rst,readdataM,readdataW);
    flopr #(32) flopenrc_27(clk,rst,aluoutM,aluoutW);
    flopr #(5) flopenrc_29(clk,rst,writeregM,writeregW);
    
    //���弶
    mux2 #(32) mumux4(.x1(aluoutW),.x2(readdataW),.a(memtoregW),.y(resultW));
    
    //ð��ģ��
    hazard myharzard(
    .stallF(startIF),
    .rsD(instrD[25:21]),
	.rtD(instrD[20:16]),
	.branchD(branchD),
	.forwardaD(forwardAD),
	.forwardbD(forwardBD),
	.stallD(startID),
	.rsE(rsE),
	.rtE(rtE),
	.writeregE(writeregE),
	.regwriteE(regwriteE),
	.memtoregE(memtoregE),
	.forwardaE(forwardAE),
	.forwardbE(forwardBE),
	.flushE(startIE),
	.writeregM(writeregM),
	.regwriteM(regwriteM),
	.memtoregM(memtoregM),
	.writeregW(writeregW),
    .regwriteW(regwriteW)
	
    );
    
endmodule    


module hazard(
	output wire stallF,
	input wire[4:0] rsD,rtD,
    input wire branchD,
	output wire forwardaD,forwardbD,
	output wire stallD,
	input wire[4:0] rsE,rtE,
	input wire[4:0] writeregE,
	input wire regwriteE,
	input wire memtoregE,
	output reg[1:0] forwardaE,forwardbE,
	output wire flushE,
	input wire[4:0] writeregM,
	input wire regwriteM,
	input wire memtoregM,
	input wire[4:0] writeregW,
	input wire regwriteW
    );
	wire lwstallD,branchstallD;
	assign forwardaD = (rsD != 0 & rsD == writeregM & regwriteM);
	assign forwardbD = (rtD != 0 & rtD == writeregM & regwriteM);
	always @(*) begin
		forwardaE = 2'b00;
		forwardbE = 2'b00;
		if(rsE != 0) begin
			if(rsE == writeregM & regwriteM) begin
				forwardaE = 2'b10;
			end else if(rsE == writeregW & regwriteW) begin
				forwardaE = 2'b01;
			end
		end
		if(rtE != 0) begin
			if(rtE == writeregM & regwriteM) begin
				forwardbE = 2'b10;
			end else if(rtE == writeregW & regwriteW) begin
				forwardbE = 2'b01;
			end
		end
	end
	assign #2 lwstallD = memtoregE & (rtE == rsD | rtE == rtD);
	assign #2 branchstallD = branchD &
				(regwriteE & 
				(writeregE == rsD | writeregE == rtD) |
				memtoregM &
				(writeregM == rsD | writeregM == rtD));
	assign #2 stallD = lwstallD | branchstallD;
	assign #2 stallF = stallD;
	assign #2 flushE = stallD;
endmodule



//��̬��֧Ԥ��ģ��
module control_dongtai (
        input wire clk,
        //Ԥ�ⲿ��
        input wire [31:0] pc,
        input wire[31:0] instrF,
        output reg pcsrcFyuce,
        output reg jumpFyuce,
        //��֤����
        input wire pcsrcD,
        input wire pcsrcDyuce,
        input wire jumpD,
        input wire jumpDyuce,
        output reg[2:0] jiucuo,
        //���±�����
        input wire[31:0] pcE
    );
    wire [7:0] suoyinF,suoyinE;
    wire pcsrcFyuceQuanjv,pcsrcFyuceJvbu;
    wire jumpFyuceQuanjv,jumpFyuceJvbu;
    wire[2:0] jiucuoCodeQuanjv,jiucuoCodeJvbu;
    wire pcsrcDyuceQuanjv,pcsrcDyuceJvbu;
    wire jumpDyuceQuanjv,jumpDyuceJvbu;
    wire pcsrcEyuceQuanjv,pcsrcEyuceJvbu;
    wire jumpEyuceQuanjv,jumpEyuceJvbu;
    wire pcsrcE;
    
    //����ȫ����ʷԤ��ģ��
     quanjv myquanjv(
        .clk(clk),
        .pc(pc),
        .instrF(instrF),
        .pcsrcFyuce(pcsrcFyuceQuanjv),//���
        .jumpFyuce(jumpFyuceQuanjv),//���
        .pcsrcD(pcsrcD),
        .pcsrcDyuce(pcsrcDyuceQuanjv),
        .jumpD(jumpD),
        .jumpDyuce(jumpDyuceQuanjv),
        .jiucuo(jiucuoCodeQuanjv),//���
        .pcE(pcE),
        //�����ǰ������ϰ����������
        .suoyinF(suoyinF),
        .suoyinE(suoyinE)
    );
    
    //����ͨ����hash����GHRƴ�ӵõ�8λ��������ȫ��Ԥ��ģ���еõ�
    //CPHT����
    cpht_256 cpht(
        .clk(clk),
	    .we3(1'b1),         //дʹ��
	    .ra1(suoyinF),      //Ҫ���ĵ�ַ 
	    .wa3(suoyinE),      //Ҫд�ĵ�ַ
	    //д��CPHT�����ݣ���λ��ʾ�ֲ���֧Ԥ���Ƿ���󣬵�λ��ʾȫ�ַ�֧Ԥ���Ƿ����
	    //E�׶���ʽд���clk����ˮ��CPU��clk��ͬ����Ϊposedge��
	    .wd3({pcsrcEyuceJvbu^pcsrcE,pcsrcEyuceQuanjv^pcsrcE}),   
	    .rd1(jishuqi)       //����������
    );
    reg swtichF;
    wire swtichD,swtichE;//ѡ���źţ�Ϊ0ʹ�þֲ���ʷ��Ϊ1ʹ��ȫ����ʷ
    always @(*) begin
        case(jishuqi)
            2'b00:swtichF<=1'b1;
            2'b01:swtichF<=1'b1;
            2'b10:swtichF<=1'b0;
            2'b11:swtichF<=1'b0;
            default:swtichF<=1'b0;
        endcase
    end
    
    //���Ӿֲ���ʷԤ��ģ��   
     jvbu myjvbu(
        .clk(clk),
        .pc(pc),
        .instrF(instrF),
        .pcsrcFyuce(pcsrcFyuceJvbu),//���
        .jumpFyuce(jumpFyuceJvbu),//���
        .pcsrcD(pcsrcD),
        .pcsrcDyuce(pcsrcDyuceJvbu),
        .jumpD(jumpD),
        .jumpDyuce(jumpDyuceJvbu),
        .jiucuo(jiucuoCodeJvbu),//���
        .pcE(pcE)
    );
    //����swtichѡ�����Ԥ���ź�
    always @(*) begin
        case(swtichF)
            1'b0:begin
                pcsrcFyuce<=pcsrcFyuceJvbu;
                jumpFyuce<=jumpFyuceJvbu;
            end
            1'b1:begin
                pcsrcFyuce<=pcsrcFyuceQuanjv;
                jumpFyuce<=jumpFyuceQuanjv;
            end
            default:begin
                pcsrcFyuce<=pcsrcFyuceJvbu;
                jumpFyuce<=jumpFyuceJvbu;
            end
        endcase
    end
    //��������������������룩
    //swtich�źż�����ˮ��
    flopr #(1) flopenrc_408(clk,1'b0,swtichF,swtichD);
    flopr #(1) flopenrc_409(clk,1'b0,swtichD,swtichE);
    //Ԥ���źż�����ˮ��
     flopr #(1) flopenrc_410(clk,1'b0,pcsrcFyuceJvbu,pcsrcDyuceJvbu);
     flopr #(1) flopenrc_411(clk,1'b0,pcsrcFyuceQuanjv,pcsrcDyuceQuanjv);
     flopr #(1) flopenrc_412(clk,1'b0,jumpFyuceJvbu,jumpDyuceJvbu);
     flopr #(1) flopenrc_413(clk,1'b0,jumpFyuceQuanjv,jumpDyuceQuanjv);
     flopr #(1) flopenrc_414(clk,1'b0,pcsrcDyuceJvbu,pcsrcEyuceJvbu);
     flopr #(1) flopenrc_415(clk,1'b0,pcsrcDyuceQuanjv,pcsrcEyuceQuanjv);
     flopr #(1) flopenrc_416(clk,1'b0,jumpDyuceJvbu,jumpEyuceJvbu);
     flopr #(1) flopenrc_417(clk,1'b0,jumpDyuceQuanjv,jumpEyuceQuanjv);
     flopr #(1) flopenrc_418(clk,1'b0,pcsrcD,pcsrcE);
     
     //����Ԥ�����
    always @(*)begin
        case(swtichD)
            1'b0:jiucuo<=jiucuoCodeJvbu;
            1'b1:jiucuo<=jiucuoCodeQuanjv;
            default:jiucuo<=jiucuoCodeJvbu;
        endcase
    end
  
      
endmodule



//��̬��֧Ԥ��ģ��
module quanjv(
        input wire clk,
        //Ԥ�ⲿ��
        input wire [31:0] pc,
        input wire[31:0] instrF,
        output reg pcsrcFyuce,
        output reg jumpFyuce,
        //��֤����
        input wire pcsrcD,
        input wire pcsrcDyuce,
        input wire jumpD,
        input wire jumpDyuce,
        output reg[2:0] jiucuo,
        //���±�����
        input wire[31:0] pcE,
        //��������
        output wire[7:0] suoyinF,
        output wire[7:0] suoyinE
    );
    
    wire [7:0] suoyinD;
    wire [2:0] ghrF;
    wire jishuqi;
    wire [2:0] ghr_read;
    wire pcsrcEyuce;
    
    //ȫ����ʷ�Ĵ���������д��һ��������
    ghr_3 myGHR(
        .clk(clk),
	    .we3(1'b1),
	    .we4(pcsrcEyuce ^ pcsrcE),//ʵ��retiredд�ص��ܶˣ�Ϊ1��ʾ��Ҫ�滻GHR
	    //E�׶��ж��Ƿ���Ҫ�滻GHR
	    .re1(1'b1),
	    .wd3(pcsrcFyuce|jumpFyuce),//Ԥ����д��
	    .wd4(ghr_read),//�滻
	    .rd1(ghrF)
    );
    //ƴ�ӵõ�����
    assign suoyinF = {pc[6:2],ghrF};
    //����pht
    pht_256 myPHT2(
        .clk(clk),
	    .we3(1'b1),      //дʹ��
	    .ra1(suoyinF),//Ҫ���ĵ�ַ 
	    .wa3(suoyinE),      //Ҫд�ĵ�ַ
	    .wd3(pcsrcE),      //д�뵼������
	    .rd1(jishuqi)       //����������
    );
    //Ԥ���֧���
    always @(*)begin
        case(instrF[31:26])
            6'b000010:jumpFyuce<=1'b1;
            default:jumpFyuce<=1'b0;
        endcase    
        case(jishuqi)
             2'b00:begin
                 pcsrcFyuce<=1'b0;
             end
             2'b01:begin
                 pcsrcFyuce<=1'b0;
             end
             2'b10:begin
                 pcsrcFyuce<=1'b1;
             end
             2'b11:begin
                 pcsrcFyuce<=1'b1;
             end
             default:begin
                 pcsrcFyuce<=1'b0;
             end
        endcase
    end
    
    
    //��֤ 
    always @(*)begin
        case({pcsrcD,pcsrcDyuce})
            2'b00:jiucuo<=3'b000;
            2'b01:jiucuo<=3'b100;
            2'b10:jiucuo<=3'b101;
            2'b11:jiucuo<=3'b000;
            default:jiucuo<=3'b000;
        endcase
    end
     
     
    //���½׶��ж��Ƿ��޸�
    //retiredGHRֻ��Ҫһ��д���we3��wd3��we4�ܶ�Ϊ0����ʾ��ʹ��
    ghr_3 retiredGHR(
        .clk(clk),
	    .we3(1'b1),
	    .we4(1'b0),
	    .re1(1'b1),//����
	    .wd3(pcsrcE),//���½׶�д����,E�׶ο�ʼд����
	    .wd4(3'b000),
	    .rd1(ghr_read)//E�׶�д��������
    );
    
    //���±���
    //������ˮ��
    flopr #(1) flopenrc_404(clk,1'b0,pcsrcD,pcsrcE);
    flopr #(8) flopenrc_406(clk,1'b0,suoyinF,suoyinD);
    flopr #(8) flopenrc_407(clk,1'b0,suoyinD,suoyinE);
    
     
endmodule





module jvbu(
        input wire clk,
        //Ԥ�ⲿ��
        input wire [31:0] pc,
        input wire[31:0] instrF,
        output reg pcsrcFyuce,
        output reg jumpFyuce,
        //��֤����
        input wire pcsrcD,
        input wire pcsrcDyuce,
        input wire jumpD,
        input wire jumpDyuce,
        output reg[2:0] jiucuo,
        //���±�����
        input wire[31:0] pcE
);
    //Ԥ��
    wire [4:0] suoyinF,suoyinE;
    wire [1:0] bhrF,bhrD,bhrE;
    wire [1:0] jishuqi,jishuqiD,jishuqiE;
    wire pcsrcE;
    assign suoyinF = pc[6:2];
   
    //����bhr
    bhr_32 myBHR(
        .clk(clk),
	    .we3(1'b1),      //дʹ��
	    .ra1(suoyinF),//Ҫ���ĵ�ַ suoyinΪ��ַ
	    .wa3(suoyinE),      //Ҫд�ĵ�ַ
	    .wd3({bhrE[0:0],pcsrcE}),      //д�뵼������
	    .rd1(bhrF)       //����������
    );
    //����bhr(�ֲ���ʷ�Ĵ���)�ҵ���Ӧ�ı��ͼ�����
    pht_4 myPHT(
        .clk(clk),
	    .we3(1'b1),      //дʹ��
	    .ra1(bhrF),//Ҫ���ĵ�ַ bhrΪ��ַ
	    .wa3(bhrE),      //Ҫд�ĵ�ַ
	    .wd3(pcsrcE),      //д�뵼������
	    .rd1(jishuqi)       //����������
    );
    //���ݼ������ж��Ƿ���ת
    always @(*) begin
        //��תָ��һ��ִ��
        case(instrF[31:26])
            6'b000010:jumpFyuce<=1'b1;
            default:jumpFyuce<=1'b0;
        endcase    
        case(jishuqi)
             2'b00:begin
                 pcsrcFyuce<=1'b0;
             end
             2'b01:begin
                 pcsrcFyuce<=1'b0;
             end
             2'b10:begin
                 pcsrcFyuce<=1'b1;
             end
             2'b11:begin
                 pcsrcFyuce<=1'b1;
             end
             default:begin
                 pcsrcFyuce<=1'b0;
             end
        endcase
    end
    
    //��֤ 
    always @(*)begin
        case({pcsrcD,pcsrcDyuce})
            2'b00:jiucuo<=3'b000;
            2'b01:jiucuo<=3'b100;
            2'b10:jiucuo<=3'b101;
            2'b11:jiucuo<=3'b000;
            default:jiucuo<=3'b000;
        endcase
    end
    //���±�
    //������ˮ��
    flopr #(2) flopenrc_400(clk,1'b0,bhrF,bhrD);
    flopr #(2) flopenrc_401(clk,1'b0,bhrD,bhrE);
    flopr #(1) flopenrc_402(clk,1'b0,pcsrcD,pcsrcE);
    assign suoyinE = pcE[6:2];
endmodule 




module pht_4(
    input wire clk,
	input wire we3,
	input wire[1:0] ra1,wa3,
	input wire wd3,
	output wire[1:0] rd1
    );
	reg [1:0] rf[3:0];
	always @(posedge clk) begin
		if(we3) begin
		     case({rf[wa3],wd3})
		         3'b000:rf[wa3]<=2'b00;
		         3'b001:rf[wa3]<=2'b01;
		         3'b010:rf[wa3]<=2'b00;
		         3'b011:rf[wa3]<=2'b11;
		         3'b100:rf[wa3]<=2'b01;
		         3'b101:rf[wa3]<=2'b11;
		         3'b110:rf[wa3]<=2'b10;
		         3'b111:rf[wa3]<=2'b11;
		         default:rf[wa3]<=rf[wa3];
		     endcase
		end
	end
	assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
endmodule

module pht_256(
    input wire clk,
	input wire we3,
	input wire[7:0] ra1,wa3,
	input wire wd3,
	output wire[1:0] rd1
    );
	reg [1:0] rf[255:0];
	always @(posedge clk) begin
		if(we3) begin
		     case({rf[wa3],wd3})
		         3'b000:rf[wa3]<=2'b00;
		         3'b001:rf[wa3]<=2'b01;
		         3'b010:rf[wa3]<=2'b00;
		         3'b011:rf[wa3]<=2'b11;
		         3'b100:rf[wa3]<=2'b01;
		         3'b101:rf[wa3]<=2'b11;
		         3'b110:rf[wa3]<=2'b10;
		         3'b111:rf[wa3]<=2'b11;
		         default:rf[wa3]<=rf[wa3];
		     endcase
		end
	end
	assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
endmodule

module bhr_32(
    input wire clk,
	input wire we3,
	input wire[4:0] ra1,wa3,
	input wire[1:0] wd3,
	output wire[1:0] rd1
);
	reg [1:0] rf[31:0];
	always @(posedge clk) begin
		if(we3) begin
			 rf[wa3] <= wd3;
		end
	end
	assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
endmodule

module ghr_3(
    input wire clk,
	input wire we3,we4,re1,//�ܶ�
	input wire wd3,//wd3���ڼ���Ԥ����
	input wire[2:0] wd4,//wd4����retiredGHR����
	output wire[2:0] rd1
);
	reg [2:0] rf;
	always @(posedge clk) begin
		if(we3) begin
			 rf <= {rf[1:0],wd3};
		end
		if(we4) begin
			 rf <= wd4;
		end
	end
	assign rd1 = (re1 != 0) ? rf : 0;
endmodule


module cpht_256(
    input wire clk,
	input wire we3,
	input wire[7:0] ra1,wa3,
	input wire [1:0] wd3,
	output wire[1:0] rd1
    );
	reg [1:0] rf[255:0];
	always @(posedge clk) begin
		if(we3) begin
		     case({rf[wa3],wd3})
		         4'b0000:rf[wa3]<=rf[wa3];
		         4'b0001:rf[wa3]<=2'b01;
		         4'b0010:rf[wa3]<=2'b00;
		         4'b0011:rf[wa3]<=rf[wa3];
		         4'b0100:rf[wa3]<=rf[wa3];
		         4'b0101:rf[wa3]<=2'b10;
		         4'b0110:rf[wa3]<=2'b00;
		         4'b0111:rf[wa3]<=rf[wa3];
		         4'b1000:rf[wa3]<=rf[wa3];
		         4'b1001:rf[wa3]<=2'b11;
		         4'b1010:rf[wa3]<=2'b01;
		         4'b1011:rf[wa3]<=rf[wa3];
		         4'b1100:rf[wa3]<=rf[wa3];
		         4'b1101:rf[wa3]<=2'b11;
		         4'b1110:rf[wa3]<=2'b10;
		         4'b1111:rf[wa3]<=rf[wa3];
		         default:rf[wa3]<=rf[wa3];
		     endcase
		end
	end
	assign rd1 = (ra1 != 0) ? rf[ra1] : 0;
endmodule

