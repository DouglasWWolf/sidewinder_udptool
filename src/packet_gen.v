//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 04-Dec-23  DWW     1  Initial creation
//====================================================================================

/*
    
*/


module packet_gen 
(
    input clk, resetn,

    //================== This is an AXI4-Lite slave interface ==================
        
    // "Specify write address"              -- Master --    -- Slave --
    input[31:0]                             S_AXI_AWADDR,   
    input                                   S_AXI_AWVALID,  
    output                                                  S_AXI_AWREADY,
    input[2:0]                              S_AXI_AWPROT,

    // "Write Data"                         -- Master --    -- Slave --
    input[31:0]                             S_AXI_WDATA,      
    input                                   S_AXI_WVALID,
    input[3:0]                              S_AXI_WSTRB,
    output                                                  S_AXI_WREADY,

    // "Send Write Response"                -- Master --    -- Slave --
    output[1:0]                                             S_AXI_BRESP,
    output                                                  S_AXI_BVALID,
    input                                   S_AXI_BREADY,

    // "Specify read address"               -- Master --    -- Slave --
    input[31:0]                             S_AXI_ARADDR,     
    input                                   S_AXI_ARVALID,
    input[2:0]                              S_AXI_ARPROT,     
    output                                                  S_AXI_ARREADY,

    // "Read data back to master"           -- Master --    -- Slave --
    output[31:0]                                            S_AXI_RDATA,
    output                                                  S_AXI_RVALID,
    output[1:0]                                             S_AXI_RRESP,
    input                                   S_AXI_RREADY,
    //==========================================================================
 

    //=========================   The output stream   ==========================
    output [511:0] AXIS_OUT_TDATA,
    output [63:0]  AXIS_OUT_TKEEP,
    output         AXIS_OUT_TLAST,
    output         AXIS_OUT_TVALID,
    input          AXIS_OUT_TREADY,
    //==========================================================================


    output reg[15:0] CYCLES_PER_PACKET
);  

    // Any time the register map of this module changes, this number should
    // be bumped
    localparam MODULE_VERSION = 1;

    //=========================  AXI Register Map  =============================
    localparam REG_MODULE_REV       = 0;
    localparam REG_COUNT_H          = 1;
    localparam REG_COUNT_L          = 2;
    localparam REG_CYCLES_PER_PKT   = 3; 
    localparam REG_PACKET_DELAY     = 4;    
    localparam REG_STATUS           = 5;
    //==========================================================================


    //==========================================================================
    // We'll communicate with the AXI4-Lite Slave core with these signals.
    //==========================================================================
    // AXI Slave Handler Interface for write requests
    wire[31:0]  ashi_waddr;     // Input:  Write-address
    wire[31:0]  ashi_wdata;     // Input:  Write-data
    wire        ashi_write;     // Input:  1 = Handle a write request
    reg[1:0]    ashi_wresp;     // Output: Write-response (OKAY, DECERR, SLVERR)
    wire        ashi_widle;     // Output: 1 = Write state machine is idle

    // AXI Slave Handler Interface for read requests
    wire[31:0]  ashi_raddr;     // Input:  Read-address
    wire        ashi_read;      // Input:  1 = Handle a read request
    reg[31:0]   ashi_rdata;     // Output: Read data
    reg[1:0]    ashi_rresp;     // Output: Read-response (OKAY, DECERR, SLVERR);
    wire        ashi_ridle;     // Output: 1 = Read state machine is idle
    //==========================================================================

    // The state of the state-machines that handle AXI4-Lite read and AXI4-Lite write
    reg[3:0] axi4_write_state, axi4_read_state;

    // The AXI4 slave state machines are idle when in state 0 and their "start" signals are low
    assign ashi_widle = (ashi_write == 0) && (axi4_write_state == 0);
    assign ashi_ridle = (ashi_read  == 0) && (axi4_read_state  == 0);
   
    // These are the valid values for ashi_rresp and ashi_wresp
    localparam OKAY   = 0;
    localparam SLVERR = 2;
    localparam DECERR = 3;

    // An AXI slave is gauranteed a minimum of 128 bytes of address space
    // (128 bytes is 32 32-bit registers)
    localparam ADDR_MASK = 7'h7F;

    reg[63:0] output_count;
    reg[31:0] packet_delay;


    // Coming out of reset, these are the default values
    localparam DEFAULT_CYCLES_PER_PKT = 3;
    localparam DEFAULT_PACKET_DELAY   = 0;

    // Strobing this high for a clock-cycle begins packet generation
    reg generate_packets;

    // When this is high, packet-generation is terminated early
    reg halt;

    // State of the (p)acket (g)eneration (s)tate (m)achine
    reg[3:0] pgsm_state;
    localparam PGSM_RESET       = 0;
    localparam PGSM_SEND_PACKET = 1;
    localparam PGSM_DELAY       = 2;

    // When this is high, the packet-generation state machine is idle
    wire pgsm_idle = (pgsm_state == PGSM_RESET) & (generate_packets == 0);


    //==========================================================================
    // pgsm - packet generation state machine
    //==========================================================================
    reg[63:0] pgsm_packet_count;
    reg[ 7:0] pgsm_cycle_count;
    reg[31:0] pgsm_delay;
    //--------------------------------------------------------------------------
    
    assign AXIS_OUT_TDATA  = {16{pgsm_packet_count[31:0]}};
    assign AXIS_OUT_TKEEP  = -1;
    assign AXIS_OUT_TLAST  = (pgsm_cycle_count == CYCLES_PER_PACKET);
    assign AXIS_OUT_TVALID = (pgsm_state == PGSM_SEND_PACKET);
    
    always @(posedge clk) begin
        
        if (resetn == 0) begin
            pgsm_state <= PGSM_RESET;
        end else case(pgsm_state)

            PGSM_RESET:
                if (generate_packets & (output_count != 0)) begin
                    pgsm_packet_count <= 1;
                    pgsm_cycle_count  <= 1; 
                    pgsm_state        <= PGSM_SEND_PACKET;
                end

            PGSM_SEND_PACKET:
                if (AXIS_OUT_TVALID & AXIS_OUT_TREADY) begin
                    pgsm_cycle_count <= pgsm_cycle_count + 1;
                    
                    if (AXIS_OUT_TLAST) begin
                        pgsm_cycle_count <= 1;
                        if (halt | (pgsm_packet_count == output_count))
                            pgsm_state <= PGSM_RESET;
                        else begin
                            pgsm_packet_count <= pgsm_packet_count + 1;
                            if (packet_delay) begin
                                pgsm_delay <= packet_delay - 1;
                                pgsm_state <= PGSM_DELAY;
                            end
                        end
                    end
                
                end 

            PGSM_DELAY:
                if (halt)
                    pgsm_state <= PGSM_RESET;
                else if (pgsm_delay)
                    pgsm_delay <= pgsm_delay - 1;
                else
                    pgsm_state <= PGSM_SEND_PACKET;           

        endcase
    end
    //==========================================================================




    //==========================================================================
    // This state machine handles AXI4-Lite write requests
    //
    // Drives:
    //==========================================================================
    always @(posedge clk) begin

        // This will strobe high to begin packet generation
        generate_packets <= 0;

        // If we're in reset, initialize important registers
        if (resetn == 0) begin
            axi4_write_state  <= 0;
            CYCLES_PER_PACKET <= DEFAULT_CYCLES_PER_PKT;
            packet_delay      <= DEFAULT_PACKET_DELAY;
            halt              <= 0;

        // If we're not in reset, and a write-request has occured...        
        end else case (axi4_write_state)
        
        0:  if (ashi_write) begin
       
                // Assume for the moment that the result will be OKAY
                ashi_wresp <= OKAY;              
            
                // Convert the byte address into a register index
                case ((ashi_waddr & ADDR_MASK) >> 2)
                

                    REG_COUNT_H:        output_count[63:32] <= ashi_wdata;
                    REG_CYCLES_PER_PKT: CYCLES_PER_PACKET   <= ashi_wdata;
                    REG_PACKET_DELAY:   packet_delay        <= ashi_wdata;

                    REG_COUNT_L:
                        if ((output_count[63:32] == 0) & (ashi_wdata == 0))
                            halt <= 1;
                        else if (pgsm_idle) begin
                            output_count[31:0] <= ashi_wdata;
                            halt               <= 0;
                            generate_packets   <= 1;
                        end

                    // Writes to any other register are a decode-error
                    default: ashi_wresp <= DECERR;
                endcase
            end

        // Dummy state, doesn't do anything
        1: axi4_write_state <= 0;

        endcase
    end
    //==========================================================================





    //==========================================================================
    // World's simplest state machine for handling AXI4-Lite read requests
    //==========================================================================
    always @(posedge clk) begin

        // If we're in reset, initialize important registers
        if (resetn == 0) begin
            axi4_read_state <= 0;
        
        // If we're not in reset, and a read-request has occured...        
        end else if (ashi_read) begin
       
            // Assume for the moment that the result will be OKAY
            ashi_rresp <= OKAY;              
            
            // Convert the byte address into a register index
            case ((ashi_raddr & ADDR_MASK) >> 2)
 
                // Allow a read from any valid register                
                REG_MODULE_REV:     ashi_rdata <= MODULE_VERSION;
                REG_COUNT_L:        ashi_rdata <= output_count[31:0];
                REG_COUNT_H:        ashi_rdata <= output_count[63:32];
                REG_CYCLES_PER_PKT: ashi_rdata <= CYCLES_PER_PACKET;
                REG_PACKET_DELAY:   ashi_rdata <= packet_delay;
                REG_STATUS:         ashi_rdata <= pgsm_idle ? 0:1;

                // Reads of any other register are a decode-error
                default: ashi_rresp <= DECERR;
            endcase
        end
    end
    //==========================================================================





    //==========================================================================
    // This connects us to an AXI4-Lite slave core
    //==========================================================================
    axi4_lite_slave axi_slave
    (
        .clk            (clk),
        .resetn         (resetn),
        
        // AXI AW channel
        .AXI_AWADDR     (S_AXI_AWADDR),
        .AXI_AWVALID    (S_AXI_AWVALID),   
        .AXI_AWPROT     (S_AXI_AWPROT),
        .AXI_AWREADY    (S_AXI_AWREADY),
        
        // AXI W channel
        .AXI_WDATA      (S_AXI_WDATA),
        .AXI_WVALID     (S_AXI_WVALID),
        .AXI_WSTRB      (S_AXI_WSTRB),
        .AXI_WREADY     (S_AXI_WREADY),

        // AXI B channel
        .AXI_BRESP      (S_AXI_BRESP),
        .AXI_BVALID     (S_AXI_BVALID),
        .AXI_BREADY     (S_AXI_BREADY),

        // AXI AR channel
        .AXI_ARADDR     (S_AXI_ARADDR), 
        .AXI_ARVALID    (S_AXI_ARVALID),
        .AXI_ARPROT     (S_AXI_ARPROT),
        .AXI_ARREADY    (S_AXI_ARREADY),

        // AXI R channel
        .AXI_RDATA      (S_AXI_RDATA),
        .AXI_RVALID     (S_AXI_RVALID),
        .AXI_RRESP      (S_AXI_RRESP),
        .AXI_RREADY     (S_AXI_RREADY),

        // ASHI write-request registers
        .ASHI_WADDR     (ashi_waddr),
        .ASHI_WDATA     (ashi_wdata),
        .ASHI_WRITE     (ashi_write),
        .ASHI_WRESP     (ashi_wresp),
        .ASHI_WIDLE     (ashi_widle),

        // ASHI read registers
        .ASHI_RADDR     (ashi_raddr),
        .ASHI_RDATA     (ashi_rdata),
        .ASHI_READ      (ashi_read ),
        .ASHI_RRESP     (ashi_rresp),
        .ASHI_RIDLE     (ashi_ridle)
    );
    //==========================================================================




endmodule
