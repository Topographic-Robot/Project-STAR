/* fpga_cam/dataRegistering.v */

module dataRegistering
  (
    /* Port Declarations */
    input       camPCLK,
    input       camVSYNC,
    input       HREF,
    input [7:0] pixData,

    output reg [15:0] pixOutput,
    output reg        buffClear1,
    output reg        buffClear2,
    output reg        writeBuff1,
    output reg        writeBuff2,
    output reg        buffSelect
  );
  
  /* _i means it is stored in a register, the non _i is just an input value and 
   * is not stored aka pipe stage */
  reg [7:0]  pixData_i;
  reg        camVSYNC_i;
  reg        HREF_i;
  reg        dataValid;
  reg [15:0] pixOutput_i;
  
  always @(posedge camPCLK)
  begin
    pixData_i  <= pixData;
    camVSYNC_i <= camVSYNC;
    HREF_i     <= HREF;
    pixOutput  <= pixOutput_i;
    
    if (!HREF_i) begin
      dataValid <= 0;
    end
    /* HREF is high */
    else begin
      dataValid <= !dataValid;
      
      if (dataValid == 0) begin
        pixOutput_i[15:8] <= pixData_i;
      end else begin
        pixOutput_i[7:0]  <= pixData_i;
      end
    end
  end
  
  reg [3:0] captureState;

  /* state assignments */
  localparam [3:0]
    IDLE        = 4'b0000,
    WAIT_WRITE1 = 4'b0001,
    WAIT_WRITE2 = 4'b0010,
    WRITE_PIX   = 4'b0011,
    NEXT_LINE1  = 4'b0100,
    NEXT_LINE2  = 4'b0101;

  /* Write-port address generator */
  always @(posedge camPCLK)
  begin
    if (camVSYNC_i) begin
      buffClear1  <= 0;
      buffClear2  <= 0;
      writeBuff1  <= 0;
      writeBuff2  <= 0;
      buffSelect  <= 0;
      captureState <= IDLE;
    end else begin
      case (captureState)
        IDLE: begin
          if (HREF_i) begin
            captureState <= WAIT_WRITE1;
            buffClear1   <= !buffSelect;
            buffClear2   <= buffSelect;
            writeBuff1   <= 0;
            writeBuff2   <= 0;
          end
        end
        
        WAIT_WRITE1: begin
          captureState <= WAIT_WRITE2;
          buffClear1   <= 0;
          buffClear2   <= 0;
          writeBuff1   <= 0;
          writeBuff2   <= 0;
        end
        
        WAIT_WRITE2: begin
          captureState <= WRITE_PIX;
          writeBuff1   <= !buffSelect;
          writeBuff2   <= buffSelect;
        end
        
        WRITE_PIX: begin
          writeBuff1   <= 0;
          writeBuff2   <= 0;
          if (!HREF_i) begin
            captureState <= NEXT_LINE1;
          end else begin
            captureState <= WAIT_WRITE2;
          end
        end
        
        NEXT_LINE1: begin
          captureState <= NEXT_LINE2;
        end
        
        NEXT_LINE2: begin
          writeBuff1   <= 0;
          writeBuff2   <= 0;
          captureState <= IDLE;
          buffSelect   <= !buffSelect;
        end
        
        default: begin
          buffClear1   <= 0;
          buffClear2   <= 0;
          writeBuff1   <= 0;
          writeBuff2   <= 0;
          buffSelect   <= 0;
          captureState <= IDLE;
        end
      endcase
    end
  end
endmodule