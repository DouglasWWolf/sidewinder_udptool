

//====================================================================================
//                        ------->  Revision History  <------
//====================================================================================
//
//   Date     Who   Ver  Changes
//====================================================================================
// 30-Nov-23  DWW     1  Initial creation
//====================================================================================


/*
     This module reads packets from the input stream and outputs it on 
     the M_AXI memory-mapped AXI interface.
*/

module shim #  
(
    parameter DATA_WBITS = 512
)
(
    input clk, resetn,

    // The number of data-cycles in a packet
    input[15:0] CYCLES_PER_PACKET,

    //=========================   The input stream   ===========================
    input [DATA_WBITS-1:0] AXIS_IN_TDATA,
    input                  AXIS_IN_TVALID,
    input                  AXIS_IN_TLAST,
    output reg             AXIS_IN_TREADY,
    //==========================================================================


    //=================   This is the AXI4 output interface   ==================

    // "Specify write address"              -- Master --    -- Slave --
    output reg [63:0]                        M_AXI_AWADDR,
    output reg [7:0]                         M_AXI_AWLEN,
    output     [2:0]                         M_AXI_AWSIZE,
    output     [3:0]                         M_AXI_AWID,
    output     [1:0]                         M_AXI_AWBURST,
    output                                   M_AXI_AWLOCK,
    output     [3:0]                         M_AXI_AWCACHE,
    output     [3:0]                         M_AXI_AWQOS,
    output     [2:0]                         M_AXI_AWPROT,
    output reg                               M_AXI_AWVALID,
    input                                                   M_AXI_AWREADY,

    // "Write Data"                         -- Master --    -- Slave --
    output reg[DATA_WBITS-1:0]              M_AXI_WDATA,
    output reg[(DATA_WBITS/8)-1:0]          M_AXI_WSTRB,
    output reg                              M_AXI_WVALID,
    output reg                              M_AXI_WLAST,
    input                                                   M_AXI_WREADY,

    // "Send Write Response"                -- Master --    -- Slave --
    input[1:0]                                              M_AXI_BRESP,
    input                                                   M_AXI_BVALID,
    output                                  M_AXI_BREADY,

    // "Specify read address"               -- Master --    -- Slave --
    output[63:0]                            M_AXI_ARADDR,
    output                                  M_AXI_ARVALID,
    output[2:0]                             M_AXI_ARPROT,
    output                                  M_AXI_ARLOCK,
    output[3:0]                             M_AXI_ARID,
    output[7:0]                             M_AXI_ARLEN,
    output[1:0]                             M_AXI_ARBURST,
    output[3:0]                             M_AXI_ARCACHE,
    output[3:0]                             M_AXI_ARQOS,
    input                                                   M_AXI_ARREADY,

    // "Read data back to master"           -- Master --    -- Slave --
    input[DATA_WBITS-1:0]                                   M_AXI_RDATA,
    input                                                   M_AXI_RVALID,
    input[1:0]                                              M_AXI_RRESP,
    input                                                   M_AXI_RLAST,
    output                                  M_AXI_RREADY
    //==========================================================================
);

// Compute the width of the data bus, in bytes
localparam DATA_WBYTS = (DATA_WBITS / 8);

// This is the target address of all of the RDMA packets we'll output
localparam FD_RING_ADDR = 64'h1111_2222_3333_4444;

// Compute the size of a data packet (which is also the size of a write-burst)
wire[15:0] packet_size = CYCLES_PER_PACKET * DATA_WBYTS;

// Offset where we'll write the next frame-data 
reg [63:0] fd_ptr;
wire[63:0] next_fd_ptr = fd_ptr + packet_size;   

// When writing data-bursts to the output interface, this is the current beat
reg[8:0] beat;

// This will be high when outputting the first beat of a burst 
wire first_beat = (M_AXI_WVALID & M_AXI_WREADY & (beat == 0));

// States of our main finite-state-machine
reg[2:0]   fsm_state;
localparam FSM_RESET       = 0;
localparam FSM_START       = 1;
localparam FSM_XFER_PACKET = 2;


//-----------------------------------------------------------------------------
// This block determines the output_mode by looking at the state of the
// main state machine
//-----------------------------------------------------------------------------

// The current output mode :
reg[2:0]   output_mode;
localparam OM_RESET = 0;
localparam OM_FD    = 1;  /* Output Mode : Frame Data              */

always @* begin
    case (fsm_state)
        FSM_XFER_PACKET:    output_mode = OM_FD;    // Frame-data
        default:            output_mode = OM_RESET;
    endcase
end
//-----------------------------------------------------------------------------


//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
// 
//     This next section manages the W-channel of the M_AXI output interface
//
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>

//-----------------------------------------------------------------------------
// Drive M_AXI_WDATA
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_WDATA = 0;
        OM_FD   :   M_AXI_WDATA = AXIS_IN_TDATA;
    endcase
end
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Drive M_AXI_WLAST
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_WLAST = 0;
        OM_FD   :   M_AXI_WLAST = AXIS_IN_TLAST;
    endcase
end
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Drive M_AXI_WSTRB
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_WSTRB = 0;
        OM_FD   :   M_AXI_WSTRB = -1;
    endcase
end
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Drive M_AXI_WVALID
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_WVALID = 0;
        OM_FD   :   M_AXI_WVALID = AXIS_IN_TVALID;
    endcase
end
//-----------------------------------------------------------------------------


//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//
//     This next section manages the AW-channel of the M_AXI output interface
// 
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>

assign M_AXI_AWID    = 0;
assign M_AXI_AWSIZE  = $clog2(DATA_WBYTS);
assign M_AXI_AWBURST = 1;       /* Burst type = Increment */
assign M_AXI_AWLOCK  = 0;       /* Not locked             */
assign M_AXI_AWCACHE = 0;       /* No caching             */
assign M_AXI_AWPROT  = 1;       /* Privileged Access      */
assign M_AXI_AWQOS   = 0;       /* No QoS                 */


//-----------------------------------------------------------------------------
// Drive M_AXI_AWADDR
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_AWADDR = 0;
        OM_FD   :   M_AXI_AWADDR = FD_RING_ADDR + fd_ptr;
    endcase
end
//-----------------------------------------------------------------------------



//-----------------------------------------------------------------------------
// Drive M_AXI_AWLEN
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_AWLEN = 0;
        OM_FD   :   M_AXI_AWLEN = CYCLES_PER_PACKET - 1;
    endcase
end
//-----------------------------------------------------------------------------


//-----------------------------------------------------------------------------
// Drive M_AXI_AWVALID - AWVALID goes active for 1 cycle when the first beat
//                       of a data burst has been accepted
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   M_AXI_AWVALID = 0;
        OM_FD   :   M_AXI_AWVALID = first_beat;
    endcase
end
//-----------------------------------------------------------------------------



//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//
//                         End of AW-channel definitions
// 
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>
//<><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><><>

// We're always ready to receive AXI write-acknowledgments
assign M_AXI_BREADY = 1;

//-----------------------------------------------------------------------------
// Drive the TREADY line of the input stream.  We only allow input when
// we're in frame-data mode and the output is ready to receive the data-cycle.
//-----------------------------------------------------------------------------
always @* begin
    case (output_mode)
        OM_RESET:   AXIS_IN_TREADY = 0;
        OM_FD   :   AXIS_IN_TREADY = M_AXI_WREADY;
    endcase
end
//-----------------------------------------------------------------------------





//=============================================================================
// This state machine is responsible for watching packets get copied from the
// input interface to the output interface and for injecting a meta-command
// packet and a frame-count packet after every frame.
//
// Drives:
//    fsm_state (and therefore, "output_mode")
//    beat
//    packet_count
//    frame_count
//=============================================================================
reg[15:0] packet_count;

always @(posedge clk) begin
    if (resetn == 0) begin
        fsm_state <= FSM_RESET;

    end else case(fsm_state)

        FSM_RESET:
            fsm_state <= FSM_START;

        FSM_START:
            begin
                beat         <= 0;
                fsm_state    <= FSM_XFER_PACKET;
            end

        FSM_XFER_PACKET:
            if (M_AXI_WVALID & M_AXI_WREADY) begin
                beat <= beat + 1;
                if (M_AXI_WLAST) begin
                    beat <= 0;
                end
            end

    endcase
end
//=============================================================================

endmodule
