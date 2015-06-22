// LPDDR2x32_example.v

// This file was auto-generated from alt_mem_if_lpddr2_tg_ed_hw.tcl.  If you edit it your changes
// will probably be lost.
// 
// Generated using ACDS version 14.1 186 at 2015.03.07.14:35:00

`timescale 1 ps / 1 ps
module LPDDR2x32_example (
//		input  wire        pll_ref_clk,              //  pll_ref_clk.clk
//		input  wire        global_reset_n,           // global_reset.reset_n
//		input  wire        soft_reset_n,             //   soft_reset.reset_n
		input	wire			CLOCK_125_p,
		output wire [9:0]  mem_ca,                   //       memory.mem_ca
		output wire [0:0]  mem_ck,                   //             .mem_ck
		output wire [0:0]  mem_ck_n,                 //             .mem_ck_n
		output wire [0:0]  mem_cke,                  //             .mem_cke
		output wire [0:0]  mem_cs_n,                 //             .mem_cs_n
		output wire [3:0]  mem_dm,                   //             .mem_dm
		inout  wire [31:0] mem_dq,                   //             .mem_dq
		inout  wire [3:0]  mem_dqs,                  //             .mem_dqs
		inout  wire [3:0]  mem_dqs_n,                //             .mem_dqs_n
		input  wire        oct_rzqin,                //          oct.rzqin
		output wire	[7:0]	LEDG
	);

	wire         pll0_pll_clk_clk;                               // pll0:pll_clk -> [d0:clk, if0:mp_cmd_clk_0_clk, if0:mp_rfifo_clk_0_clk, if0:mp_wfifo_clk_0_clk, mm_interconnect_0:pll0_pll_clk_clk, rst_controller_004:clk]
	wire         d0_avl_beginbursttransfer;                      // d0:avl_burstbegin -> mm_interconnect_0:d0_avl_beginbursttransfer
	wire         d0_avl_waitrequest;                             // mm_interconnect_0:d0_avl_waitrequest -> d0:avl_ready
	wire  [31:0] d0_avl_readdata;                                // mm_interconnect_0:d0_avl_readdata -> d0:avl_rdata
	wire  [28:0] d0_avl_address;                                 // d0:avl_addr -> mm_interconnect_0:d0_avl_address
	wire         d0_avl_read;                                    // d0:avl_read_req -> mm_interconnect_0:d0_avl_read
	wire   [3:0] d0_avl_byteenable;                              // d0:avl_be -> mm_interconnect_0:d0_avl_byteenable
	wire         d0_avl_readdatavalid;                           // mm_interconnect_0:d0_avl_readdatavalid -> d0:avl_rdata_valid
	wire  [31:0] d0_avl_writedata;                               // d0:avl_wdata -> mm_interconnect_0:d0_avl_writedata
	wire         d0_avl_write;                                   // d0:avl_write_req -> mm_interconnect_0:d0_avl_write
	wire   [2:0] d0_avl_burstcount;                              // d0:avl_size -> mm_interconnect_0:d0_avl_burstcount
	wire         mm_interconnect_0_if0_avl_0_beginbursttransfer; // mm_interconnect_0:if0_avl_0_beginbursttransfer -> if0:avl_burstbegin_0
	wire  [31:0] mm_interconnect_0_if0_avl_0_readdata;           // if0:avl_rdata_0 -> mm_interconnect_0:if0_avl_0_readdata
	wire         mm_interconnect_0_if0_avl_0_waitrequest;        // if0:avl_ready_0 -> mm_interconnect_0:if0_avl_0_waitrequest
	wire  [26:0] mm_interconnect_0_if0_avl_0_address;            // mm_interconnect_0:if0_avl_0_address -> if0:avl_addr_0
	wire         mm_interconnect_0_if0_avl_0_read;               // mm_interconnect_0:if0_avl_0_read -> if0:avl_read_req_0
	wire   [3:0] mm_interconnect_0_if0_avl_0_byteenable;         // mm_interconnect_0:if0_avl_0_byteenable -> if0:avl_be_0
	wire         mm_interconnect_0_if0_avl_0_readdatavalid;      // if0:avl_rdata_valid_0 -> mm_interconnect_0:if0_avl_0_readdatavalid
	wire         mm_interconnect_0_if0_avl_0_write;              // mm_interconnect_0:if0_avl_0_write -> if0:avl_write_req_0
	wire  [31:0] mm_interconnect_0_if0_avl_0_writedata;          // mm_interconnect_0:if0_avl_0_writedata -> if0:avl_wdata_0
	wire   [2:0] mm_interconnect_0_if0_avl_0_burstcount;         // mm_interconnect_0:if0_avl_0_burstcount -> if0:avl_size_0
	wire         rst_controller_reset_out_reset;                 // rst_controller:reset_out -> if0:mp_cmd_reset_n_0_reset_n
	wire         pll0_reset_out_reset;                           // pll0:reset_out_n -> [rst_controller:reset_in0, rst_controller_001:reset_in0, rst_controller_002:reset_in0, rst_controller_003:reset_in0, rst_controller_004:reset_in0]
	wire         rst_controller_001_reset_out_reset;             // rst_controller_001:reset_out -> if0:mp_rfifo_reset_n_0_reset_n
	wire         rst_controller_002_reset_out_reset;             // rst_controller_002:reset_out -> if0:mp_wfifo_reset_n_0_reset_n
	wire         rst_controller_003_reset_out_reset;             // rst_controller_003:reset_out -> d0:reset_n
	wire         rst_controller_004_reset_out_reset;             // rst_controller_004:reset_out -> [mm_interconnect_0:d0_avl_reset_reset_bridge_in_reset_reset, mm_interconnect_0:d0_avl_translator_reset_reset_bridge_in_reset_reset]

	wire pll_locked;
	reg global_reset_n, soft_reset_n;
	wire [1:0] resets_n;
	reg [1:0] resets_n_reg;
	wire drv_status_pass, drv_status_fail, drv_status_test_complete;
	wire local_cal_fail, local_cal_success, local_init_done;
	reg drv_status_pass_reg, drv_status_fail_reg, drv_status_test_complete_reg;
	reg local_init_done_reg, local_cal_success_reg, local_cal_fail_reg;
	
	
	ISSP ISSP_inst(
		.source_clk(CLOCK_125_p),
		.source(resets_n),
		.probe({drv_status_pass_reg, drv_status_fail_reg, drv_status_test_complete_reg,
					local_init_done_reg, local_cal_success_reg, local_cal_fail_reg, pll_locked})
	);
	
	assign LEDG[6:0] = {drv_status_pass_reg, drv_status_fail_reg, drv_status_test_complete_reg,
					local_init_done_reg, local_cal_success_reg, local_cal_fail_reg, pll_locked};
					
	always @(posedge CLOCK_125_p) begin
		drv_status_pass_reg <= drv_status_pass;
		drv_status_fail_reg <= drv_status_fail;
		drv_status_test_complete_reg <= drv_status_test_complete;
		local_init_done_reg <= local_init_done;
		local_cal_success_reg <= local_cal_success;
		local_cal_fail_reg <= local_cal_fail;
		resets_n_reg <= resets_n;
		global_reset_n <= resets_n_reg[0];
		soft_reset_n <= resets_n_reg[1];
	end
	
	LPDDR2x32_example_if0 if0 (
		.pll_ref_clk                (CLOCK_125_p),                                    //        pll_ref_clk.clk
		.global_reset_n             (global_reset_n),                                 //       global_reset.reset_n
		.soft_reset_n               (soft_reset_n),                                   //         soft_reset.reset_n
		.afi_clk                    (),                                               //            afi_clk.clk
		.afi_half_clk               (),                                               //       afi_half_clk.clk
		.afi_reset_n                (),                                               //          afi_reset.reset_n
		.afi_reset_export_n         (),                                               //   afi_reset_export.reset_n
		.mem_ca                     (mem_ca),                                         //             memory.mem_ca
		.mem_ck                     (mem_ck),                                         //                   .mem_ck
		.mem_ck_n                   (mem_ck_n),                                       //                   .mem_ck_n
		.mem_cke                    (mem_cke),                                        //                   .mem_cke
		.mem_cs_n                   (mem_cs_n),                                       //                   .mem_cs_n
		.mem_dm                     (mem_dm),                                         //                   .mem_dm
		.mem_dq                     (mem_dq),                                         //                   .mem_dq
		.mem_dqs                    (mem_dqs),                                        //                   .mem_dqs
		.mem_dqs_n                  (mem_dqs_n),                                      //                   .mem_dqs_n
		.avl_ready_0                (mm_interconnect_0_if0_avl_0_waitrequest),        //              avl_0.waitrequest_n
		.avl_burstbegin_0           (mm_interconnect_0_if0_avl_0_beginbursttransfer), //                   .beginbursttransfer
		.avl_addr_0                 (mm_interconnect_0_if0_avl_0_address),            //                   .address
		.avl_rdata_valid_0          (mm_interconnect_0_if0_avl_0_readdatavalid),      //                   .readdatavalid
		.avl_rdata_0                (mm_interconnect_0_if0_avl_0_readdata),           //                   .readdata
		.avl_wdata_0                (mm_interconnect_0_if0_avl_0_writedata),          //                   .writedata
		.avl_be_0                   (mm_interconnect_0_if0_avl_0_byteenable),         //                   .byteenable
		.avl_read_req_0             (mm_interconnect_0_if0_avl_0_read),               //                   .read
		.avl_write_req_0            (mm_interconnect_0_if0_avl_0_write),              //                   .write
		.avl_size_0                 (mm_interconnect_0_if0_avl_0_burstcount),         //                   .burstcount
		.mp_cmd_clk_0_clk           (pll0_pll_clk_clk),                               //       mp_cmd_clk_0.clk
		.mp_cmd_reset_n_0_reset_n   (~rst_controller_reset_out_reset),                //   mp_cmd_reset_n_0.reset_n
		.mp_rfifo_clk_0_clk         (pll0_pll_clk_clk),                               //     mp_rfifo_clk_0.clk
		.mp_rfifo_reset_n_0_reset_n (~rst_controller_001_reset_out_reset),            // mp_rfifo_reset_n_0.reset_n
		.mp_wfifo_clk_0_clk         (pll0_pll_clk_clk),                               //     mp_wfifo_clk_0.clk
		.mp_wfifo_reset_n_0_reset_n (~rst_controller_002_reset_out_reset),            // mp_wfifo_reset_n_0.reset_n
		.local_init_done            (local_init_done),                                //             status.local_init_done
		.local_cal_success          (local_cal_success),                              //                   .local_cal_success
		.local_cal_fail             (local_cal_fail),                                 //                   .local_cal_fail
		.oct_rzqin                  (oct_rzqin),                                      //                oct.rzqin
		.pll_mem_clk                (),                                               //        pll_sharing.pll_mem_clk
		.pll_write_clk              (),                                               //                   .pll_write_clk
		.pll_locked                 (),                                               //                   .pll_locked
		.pll_write_clk_pre_phy_clk  (),                                               //                   .pll_write_clk_pre_phy_clk
		.pll_addr_cmd_clk           (),                                               //                   .pll_addr_cmd_clk
		.pll_avl_clk                (),                                               //                   .pll_avl_clk
		.pll_config_clk             (),                                               //                   .pll_config_clk
		.pll_mem_phy_clk            (),                                               //                   .pll_mem_phy_clk
		.afi_phy_clk                (),                                               //                   .afi_phy_clk
		.pll_avl_phy_clk            ()                                                //                   .pll_avl_phy_clk
	);

	altera_mem_if_single_clock_pll #(
		.DEVICE_FAMILY    ("Cyclone V"),
		.REF_CLK_FREQ_STR ("125.0 MHz"),
		.REF_CLK_PS       ("8000.0"),
		.PLL_CLK_FREQ_STR ("125.0 MHz"),
		.PLL_CLK_PHASE_PS (0),
		.PLL_CLK_MULT     (0),
		.PLL_CLK_DIV      (0),
		.USE_GENERIC_PLL  (1)
	) pll0 (
		.pll_ref_clk    (CLOCK_125_p),          //    pll_ref_clk.clk
		.pll_clk        (pll0_pll_clk_clk),     //        pll_clk.clk
		.global_reset_n (global_reset_n),       // global_reset_n.reset_n
		.pll_locked     (pll_locked),                     //     pll_locked.pll_locked
		.reset_out_n    (pll0_reset_out_reset)  //      reset_out.reset_n
	);

	LPDDR2x32_example_d0 #(
		.DEVICE_FAMILY                          ("Cyclone V"),
		.TG_AVL_DATA_WIDTH                      (32),
		.TG_AVL_ADDR_WIDTH                      (29),
		.TG_AVL_WORD_ADDR_WIDTH                 (27),
		.TG_AVL_SIZE_WIDTH                      (3),
		.TG_AVL_BE_WIDTH                        (4),
		.DRIVER_SIGNATURE                       (1431634061),
		.TG_GEN_BYTE_ADDR                       (1),
		.TG_NUM_DRIVER_LOOP                     (1),
		.TG_ENABLE_UNIX_ID                      (0),
		.TG_USE_UNIX_ID                         (0),
		.TG_RANDOM_BYTE_ENABLE                  (1),
		.TG_ENABLE_READ_COMPARE                 (1),
		.TG_POWER_OF_TWO_BURSTS_ONLY            (0),
		.TG_BURST_ON_BURST_BOUNDARY             (0),
		.TG_DO_NOT_CROSS_4KB_BOUNDARY           (0),
		.TG_TIMEOUT_COUNTER_WIDTH               (32),
		.TG_MAX_READ_LATENCY                    (20),
		.TG_SINGLE_RW_SEQ_ADDR_COUNT            (32),
		.TG_SINGLE_RW_RAND_ADDR_COUNT           (32),
		.TG_SINGLE_RW_RAND_SEQ_ADDR_COUNT       (32),
		.TG_BLOCK_RW_SEQ_ADDR_COUNT             (8),
		.TG_BLOCK_RW_RAND_ADDR_COUNT            (8),
		.TG_BLOCK_RW_RAND_SEQ_ADDR_COUNT        (8),
		.TG_BLOCK_RW_BLOCK_SIZE                 (8),
		.TG_TEMPLATE_STAGE_COUNT                (4),
		.TG_SEQ_ADDR_GEN_MIN_BURSTCOUNT         (1),
		.TG_SEQ_ADDR_GEN_MAX_BURSTCOUNT         (4),
		.TG_RAND_ADDR_GEN_MIN_BURSTCOUNT        (1),
		.TG_RAND_ADDR_GEN_MAX_BURSTCOUNT        (4),
		.TG_RAND_SEQ_ADDR_GEN_MIN_BURSTCOUNT    (1),
		.TG_RAND_SEQ_ADDR_GEN_MAX_BURSTCOUNT    (4),
		.TG_RAND_SEQ_ADDR_GEN_RAND_ADDR_PERCENT (50)
	) d0 (
		.clk             (pll0_pll_clk_clk),                    // avl_clock.clk
		.reset_n         (~rst_controller_003_reset_out_reset), // avl_reset.reset_n
		.pass            (drv_status_pass),                     //    status.pass
		.fail            (drv_status_fail),                     //          .fail
		.test_complete   (drv_status_test_complete),            //          .test_complete
		.avl_ready       (~d0_avl_waitrequest),                 //       avl.waitrequest_n
		.avl_addr        (d0_avl_address),                      //          .address
		.avl_size        (d0_avl_burstcount),                   //          .burstcount
		.avl_wdata       (d0_avl_writedata),                    //          .writedata
		.avl_rdata       (d0_avl_readdata),                     //          .readdata
		.avl_write_req   (d0_avl_write),                        //          .write
		.avl_read_req    (d0_avl_read),                         //          .read
		.avl_rdata_valid (d0_avl_readdatavalid),                //          .readdatavalid
		.avl_be          (d0_avl_byteenable),                   //          .byteenable
		.avl_burstbegin  (d0_avl_beginbursttransfer)            //          .beginbursttransfer
	);

	LPDDR2x32_example_mm_interconnect_0 mm_interconnect_0 (
		.pll0_pll_clk_clk                                    (pll0_pll_clk_clk),                               //                                  pll0_pll_clk.clk
		.d0_avl_reset_reset_bridge_in_reset_reset            (rst_controller_004_reset_out_reset),             //            d0_avl_reset_reset_bridge_in_reset.reset
		.d0_avl_translator_reset_reset_bridge_in_reset_reset (rst_controller_004_reset_out_reset),             // d0_avl_translator_reset_reset_bridge_in_reset.reset
		.d0_avl_address                                      (d0_avl_address),                                 //                                        d0_avl.address
		.d0_avl_waitrequest                                  (d0_avl_waitrequest),                             //                                              .waitrequest
		.d0_avl_burstcount                                   (d0_avl_burstcount),                              //                                              .burstcount
		.d0_avl_byteenable                                   (d0_avl_byteenable),                              //                                              .byteenable
		.d0_avl_beginbursttransfer                           (d0_avl_beginbursttransfer),                      //                                              .beginbursttransfer
		.d0_avl_read                                         (d0_avl_read),                                    //                                              .read
		.d0_avl_readdata                                     (d0_avl_readdata),                                //                                              .readdata
		.d0_avl_readdatavalid                                (d0_avl_readdatavalid),                           //                                              .readdatavalid
		.d0_avl_write                                        (d0_avl_write),                                   //                                              .write
		.d0_avl_writedata                                    (d0_avl_writedata),                               //                                              .writedata
		.if0_avl_0_address                                   (mm_interconnect_0_if0_avl_0_address),            //                                     if0_avl_0.address
		.if0_avl_0_write                                     (mm_interconnect_0_if0_avl_0_write),              //                                              .write
		.if0_avl_0_read                                      (mm_interconnect_0_if0_avl_0_read),               //                                              .read
		.if0_avl_0_readdata                                  (mm_interconnect_0_if0_avl_0_readdata),           //                                              .readdata
		.if0_avl_0_writedata                                 (mm_interconnect_0_if0_avl_0_writedata),          //                                              .writedata
		.if0_avl_0_beginbursttransfer                        (mm_interconnect_0_if0_avl_0_beginbursttransfer), //                                              .beginbursttransfer
		.if0_avl_0_burstcount                                (mm_interconnect_0_if0_avl_0_burstcount),         //                                              .burstcount
		.if0_avl_0_byteenable                                (mm_interconnect_0_if0_avl_0_byteenable),         //                                              .byteenable
		.if0_avl_0_readdatavalid                             (mm_interconnect_0_if0_avl_0_readdatavalid),      //                                              .readdatavalid
		.if0_avl_0_waitrequest                               (~mm_interconnect_0_if0_avl_0_waitrequest)        //                                              .waitrequest
	);

	altera_reset_controller #(
		.NUM_RESET_INPUTS          (3),
		.OUTPUT_RESET_SYNC_EDGES   ("none"),
		.SYNC_DEPTH                (2),
		.RESET_REQUEST_PRESENT     (0),
		.RESET_REQ_WAIT_TIME       (1),
		.MIN_RST_ASSERTION_TIME    (3),
		.RESET_REQ_EARLY_DSRT_TIME (1),
		.USE_RESET_REQUEST_IN0     (0),
		.USE_RESET_REQUEST_IN1     (0),
		.USE_RESET_REQUEST_IN2     (0),
		.USE_RESET_REQUEST_IN3     (0),
		.USE_RESET_REQUEST_IN4     (0),
		.USE_RESET_REQUEST_IN5     (0),
		.USE_RESET_REQUEST_IN6     (0),
		.USE_RESET_REQUEST_IN7     (0),
		.USE_RESET_REQUEST_IN8     (0),
		.USE_RESET_REQUEST_IN9     (0),
		.USE_RESET_REQUEST_IN10    (0),
		.USE_RESET_REQUEST_IN11    (0),
		.USE_RESET_REQUEST_IN12    (0),
		.USE_RESET_REQUEST_IN13    (0),
		.USE_RESET_REQUEST_IN14    (0),
		.USE_RESET_REQUEST_IN15    (0),
		.ADAPT_RESET_REQUEST       (0)
	) rst_controller (
		.reset_in0      (~pll0_reset_out_reset),          // reset_in0.reset
		.reset_in1      (~global_reset_n),                // reset_in1.reset
		.reset_in2      (~soft_reset_n),                  // reset_in2.reset
		.clk            (),                               //       clk.clk
		.reset_out      (rst_controller_reset_out_reset), // reset_out.reset
		.reset_req      (),                               // (terminated)
		.reset_req_in0  (1'b0),                           // (terminated)
		.reset_req_in1  (1'b0),                           // (terminated)
		.reset_req_in2  (1'b0),                           // (terminated)
		.reset_in3      (1'b0),                           // (terminated)
		.reset_req_in3  (1'b0),                           // (terminated)
		.reset_in4      (1'b0),                           // (terminated)
		.reset_req_in4  (1'b0),                           // (terminated)
		.reset_in5      (1'b0),                           // (terminated)
		.reset_req_in5  (1'b0),                           // (terminated)
		.reset_in6      (1'b0),                           // (terminated)
		.reset_req_in6  (1'b0),                           // (terminated)
		.reset_in7      (1'b0),                           // (terminated)
		.reset_req_in7  (1'b0),                           // (terminated)
		.reset_in8      (1'b0),                           // (terminated)
		.reset_req_in8  (1'b0),                           // (terminated)
		.reset_in9      (1'b0),                           // (terminated)
		.reset_req_in9  (1'b0),                           // (terminated)
		.reset_in10     (1'b0),                           // (terminated)
		.reset_req_in10 (1'b0),                           // (terminated)
		.reset_in11     (1'b0),                           // (terminated)
		.reset_req_in11 (1'b0),                           // (terminated)
		.reset_in12     (1'b0),                           // (terminated)
		.reset_req_in12 (1'b0),                           // (terminated)
		.reset_in13     (1'b0),                           // (terminated)
		.reset_req_in13 (1'b0),                           // (terminated)
		.reset_in14     (1'b0),                           // (terminated)
		.reset_req_in14 (1'b0),                           // (terminated)
		.reset_in15     (1'b0),                           // (terminated)
		.reset_req_in15 (1'b0)                            // (terminated)
	);

	altera_reset_controller #(
		.NUM_RESET_INPUTS          (3),
		.OUTPUT_RESET_SYNC_EDGES   ("none"),
		.SYNC_DEPTH                (2),
		.RESET_REQUEST_PRESENT     (0),
		.RESET_REQ_WAIT_TIME       (1),
		.MIN_RST_ASSERTION_TIME    (3),
		.RESET_REQ_EARLY_DSRT_TIME (1),
		.USE_RESET_REQUEST_IN0     (0),
		.USE_RESET_REQUEST_IN1     (0),
		.USE_RESET_REQUEST_IN2     (0),
		.USE_RESET_REQUEST_IN3     (0),
		.USE_RESET_REQUEST_IN4     (0),
		.USE_RESET_REQUEST_IN5     (0),
		.USE_RESET_REQUEST_IN6     (0),
		.USE_RESET_REQUEST_IN7     (0),
		.USE_RESET_REQUEST_IN8     (0),
		.USE_RESET_REQUEST_IN9     (0),
		.USE_RESET_REQUEST_IN10    (0),
		.USE_RESET_REQUEST_IN11    (0),
		.USE_RESET_REQUEST_IN12    (0),
		.USE_RESET_REQUEST_IN13    (0),
		.USE_RESET_REQUEST_IN14    (0),
		.USE_RESET_REQUEST_IN15    (0),
		.ADAPT_RESET_REQUEST       (0)
	) rst_controller_001 (
		.reset_in0      (~pll0_reset_out_reset),              // reset_in0.reset
		.reset_in1      (~global_reset_n),                    // reset_in1.reset
		.reset_in2      (~soft_reset_n),                      // reset_in2.reset
		.clk            (),                                   //       clk.clk
		.reset_out      (rst_controller_001_reset_out_reset), // reset_out.reset
		.reset_req      (),                                   // (terminated)
		.reset_req_in0  (1'b0),                               // (terminated)
		.reset_req_in1  (1'b0),                               // (terminated)
		.reset_req_in2  (1'b0),                               // (terminated)
		.reset_in3      (1'b0),                               // (terminated)
		.reset_req_in3  (1'b0),                               // (terminated)
		.reset_in4      (1'b0),                               // (terminated)
		.reset_req_in4  (1'b0),                               // (terminated)
		.reset_in5      (1'b0),                               // (terminated)
		.reset_req_in5  (1'b0),                               // (terminated)
		.reset_in6      (1'b0),                               // (terminated)
		.reset_req_in6  (1'b0),                               // (terminated)
		.reset_in7      (1'b0),                               // (terminated)
		.reset_req_in7  (1'b0),                               // (terminated)
		.reset_in8      (1'b0),                               // (terminated)
		.reset_req_in8  (1'b0),                               // (terminated)
		.reset_in9      (1'b0),                               // (terminated)
		.reset_req_in9  (1'b0),                               // (terminated)
		.reset_in10     (1'b0),                               // (terminated)
		.reset_req_in10 (1'b0),                               // (terminated)
		.reset_in11     (1'b0),                               // (terminated)
		.reset_req_in11 (1'b0),                               // (terminated)
		.reset_in12     (1'b0),                               // (terminated)
		.reset_req_in12 (1'b0),                               // (terminated)
		.reset_in13     (1'b0),                               // (terminated)
		.reset_req_in13 (1'b0),                               // (terminated)
		.reset_in14     (1'b0),                               // (terminated)
		.reset_req_in14 (1'b0),                               // (terminated)
		.reset_in15     (1'b0),                               // (terminated)
		.reset_req_in15 (1'b0)                                // (terminated)
	);

	altera_reset_controller #(
		.NUM_RESET_INPUTS          (3),
		.OUTPUT_RESET_SYNC_EDGES   ("none"),
		.SYNC_DEPTH                (2),
		.RESET_REQUEST_PRESENT     (0),
		.RESET_REQ_WAIT_TIME       (1),
		.MIN_RST_ASSERTION_TIME    (3),
		.RESET_REQ_EARLY_DSRT_TIME (1),
		.USE_RESET_REQUEST_IN0     (0),
		.USE_RESET_REQUEST_IN1     (0),
		.USE_RESET_REQUEST_IN2     (0),
		.USE_RESET_REQUEST_IN3     (0),
		.USE_RESET_REQUEST_IN4     (0),
		.USE_RESET_REQUEST_IN5     (0),
		.USE_RESET_REQUEST_IN6     (0),
		.USE_RESET_REQUEST_IN7     (0),
		.USE_RESET_REQUEST_IN8     (0),
		.USE_RESET_REQUEST_IN9     (0),
		.USE_RESET_REQUEST_IN10    (0),
		.USE_RESET_REQUEST_IN11    (0),
		.USE_RESET_REQUEST_IN12    (0),
		.USE_RESET_REQUEST_IN13    (0),
		.USE_RESET_REQUEST_IN14    (0),
		.USE_RESET_REQUEST_IN15    (0),
		.ADAPT_RESET_REQUEST       (0)
	) rst_controller_002 (
		.reset_in0      (~pll0_reset_out_reset),              // reset_in0.reset
		.reset_in1      (~global_reset_n),                    // reset_in1.reset
		.reset_in2      (~soft_reset_n),                      // reset_in2.reset
		.clk            (),                                   //       clk.clk
		.reset_out      (rst_controller_002_reset_out_reset), // reset_out.reset
		.reset_req      (),                                   // (terminated)
		.reset_req_in0  (1'b0),                               // (terminated)
		.reset_req_in1  (1'b0),                               // (terminated)
		.reset_req_in2  (1'b0),                               // (terminated)
		.reset_in3      (1'b0),                               // (terminated)
		.reset_req_in3  (1'b0),                               // (terminated)
		.reset_in4      (1'b0),                               // (terminated)
		.reset_req_in4  (1'b0),                               // (terminated)
		.reset_in5      (1'b0),                               // (terminated)
		.reset_req_in5  (1'b0),                               // (terminated)
		.reset_in6      (1'b0),                               // (terminated)
		.reset_req_in6  (1'b0),                               // (terminated)
		.reset_in7      (1'b0),                               // (terminated)
		.reset_req_in7  (1'b0),                               // (terminated)
		.reset_in8      (1'b0),                               // (terminated)
		.reset_req_in8  (1'b0),                               // (terminated)
		.reset_in9      (1'b0),                               // (terminated)
		.reset_req_in9  (1'b0),                               // (terminated)
		.reset_in10     (1'b0),                               // (terminated)
		.reset_req_in10 (1'b0),                               // (terminated)
		.reset_in11     (1'b0),                               // (terminated)
		.reset_req_in11 (1'b0),                               // (terminated)
		.reset_in12     (1'b0),                               // (terminated)
		.reset_req_in12 (1'b0),                               // (terminated)
		.reset_in13     (1'b0),                               // (terminated)
		.reset_req_in13 (1'b0),                               // (terminated)
		.reset_in14     (1'b0),                               // (terminated)
		.reset_req_in14 (1'b0),                               // (terminated)
		.reset_in15     (1'b0),                               // (terminated)
		.reset_req_in15 (1'b0)                                // (terminated)
	);

	altera_reset_controller #(
		.NUM_RESET_INPUTS          (3),
		.OUTPUT_RESET_SYNC_EDGES   ("none"),
		.SYNC_DEPTH                (2),
		.RESET_REQUEST_PRESENT     (0),
		.RESET_REQ_WAIT_TIME       (1),
		.MIN_RST_ASSERTION_TIME    (3),
		.RESET_REQ_EARLY_DSRT_TIME (1),
		.USE_RESET_REQUEST_IN0     (0),
		.USE_RESET_REQUEST_IN1     (0),
		.USE_RESET_REQUEST_IN2     (0),
		.USE_RESET_REQUEST_IN3     (0),
		.USE_RESET_REQUEST_IN4     (0),
		.USE_RESET_REQUEST_IN5     (0),
		.USE_RESET_REQUEST_IN6     (0),
		.USE_RESET_REQUEST_IN7     (0),
		.USE_RESET_REQUEST_IN8     (0),
		.USE_RESET_REQUEST_IN9     (0),
		.USE_RESET_REQUEST_IN10    (0),
		.USE_RESET_REQUEST_IN11    (0),
		.USE_RESET_REQUEST_IN12    (0),
		.USE_RESET_REQUEST_IN13    (0),
		.USE_RESET_REQUEST_IN14    (0),
		.USE_RESET_REQUEST_IN15    (0),
		.ADAPT_RESET_REQUEST       (0)
	) rst_controller_003 (
		.reset_in0      (~pll0_reset_out_reset),              // reset_in0.reset
		.reset_in1      (~global_reset_n),                    // reset_in1.reset
		.reset_in2      (~soft_reset_n),                      // reset_in2.reset
		.clk            (),                                   //       clk.clk
		.reset_out      (rst_controller_003_reset_out_reset), // reset_out.reset
		.reset_req      (),                                   // (terminated)
		.reset_req_in0  (1'b0),                               // (terminated)
		.reset_req_in1  (1'b0),                               // (terminated)
		.reset_req_in2  (1'b0),                               // (terminated)
		.reset_in3      (1'b0),                               // (terminated)
		.reset_req_in3  (1'b0),                               // (terminated)
		.reset_in4      (1'b0),                               // (terminated)
		.reset_req_in4  (1'b0),                               // (terminated)
		.reset_in5      (1'b0),                               // (terminated)
		.reset_req_in5  (1'b0),                               // (terminated)
		.reset_in6      (1'b0),                               // (terminated)
		.reset_req_in6  (1'b0),                               // (terminated)
		.reset_in7      (1'b0),                               // (terminated)
		.reset_req_in7  (1'b0),                               // (terminated)
		.reset_in8      (1'b0),                               // (terminated)
		.reset_req_in8  (1'b0),                               // (terminated)
		.reset_in9      (1'b0),                               // (terminated)
		.reset_req_in9  (1'b0),                               // (terminated)
		.reset_in10     (1'b0),                               // (terminated)
		.reset_req_in10 (1'b0),                               // (terminated)
		.reset_in11     (1'b0),                               // (terminated)
		.reset_req_in11 (1'b0),                               // (terminated)
		.reset_in12     (1'b0),                               // (terminated)
		.reset_req_in12 (1'b0),                               // (terminated)
		.reset_in13     (1'b0),                               // (terminated)
		.reset_req_in13 (1'b0),                               // (terminated)
		.reset_in14     (1'b0),                               // (terminated)
		.reset_req_in14 (1'b0),                               // (terminated)
		.reset_in15     (1'b0),                               // (terminated)
		.reset_req_in15 (1'b0)                                // (terminated)
	);

	altera_reset_controller #(
		.NUM_RESET_INPUTS          (3),
		.OUTPUT_RESET_SYNC_EDGES   ("deassert"),
		.SYNC_DEPTH                (2),
		.RESET_REQUEST_PRESENT     (0),
		.RESET_REQ_WAIT_TIME       (1),
		.MIN_RST_ASSERTION_TIME    (3),
		.RESET_REQ_EARLY_DSRT_TIME (1),
		.USE_RESET_REQUEST_IN0     (0),
		.USE_RESET_REQUEST_IN1     (0),
		.USE_RESET_REQUEST_IN2     (0),
		.USE_RESET_REQUEST_IN3     (0),
		.USE_RESET_REQUEST_IN4     (0),
		.USE_RESET_REQUEST_IN5     (0),
		.USE_RESET_REQUEST_IN6     (0),
		.USE_RESET_REQUEST_IN7     (0),
		.USE_RESET_REQUEST_IN8     (0),
		.USE_RESET_REQUEST_IN9     (0),
		.USE_RESET_REQUEST_IN10    (0),
		.USE_RESET_REQUEST_IN11    (0),
		.USE_RESET_REQUEST_IN12    (0),
		.USE_RESET_REQUEST_IN13    (0),
		.USE_RESET_REQUEST_IN14    (0),
		.USE_RESET_REQUEST_IN15    (0),
		.ADAPT_RESET_REQUEST       (0)
	) rst_controller_004 (
		.reset_in0      (~pll0_reset_out_reset),              // reset_in0.reset
		.reset_in1      (~global_reset_n),                    // reset_in1.reset
		.reset_in2      (~soft_reset_n),                      // reset_in2.reset
		.clk            (pll0_pll_clk_clk),                   //       clk.clk
		.reset_out      (rst_controller_004_reset_out_reset), // reset_out.reset
		.reset_req      (),                                   // (terminated)
		.reset_req_in0  (1'b0),                               // (terminated)
		.reset_req_in1  (1'b0),                               // (terminated)
		.reset_req_in2  (1'b0),                               // (terminated)
		.reset_in3      (1'b0),                               // (terminated)
		.reset_req_in3  (1'b0),                               // (terminated)
		.reset_in4      (1'b0),                               // (terminated)
		.reset_req_in4  (1'b0),                               // (terminated)
		.reset_in5      (1'b0),                               // (terminated)
		.reset_req_in5  (1'b0),                               // (terminated)
		.reset_in6      (1'b0),                               // (terminated)
		.reset_req_in6  (1'b0),                               // (terminated)
		.reset_in7      (1'b0),                               // (terminated)
		.reset_req_in7  (1'b0),                               // (terminated)
		.reset_in8      (1'b0),                               // (terminated)
		.reset_req_in8  (1'b0),                               // (terminated)
		.reset_in9      (1'b0),                               // (terminated)
		.reset_req_in9  (1'b0),                               // (terminated)
		.reset_in10     (1'b0),                               // (terminated)
		.reset_req_in10 (1'b0),                               // (terminated)
		.reset_in11     (1'b0),                               // (terminated)
		.reset_req_in11 (1'b0),                               // (terminated)
		.reset_in12     (1'b0),                               // (terminated)
		.reset_req_in12 (1'b0),                               // (terminated)
		.reset_in13     (1'b0),                               // (terminated)
		.reset_req_in13 (1'b0),                               // (terminated)
		.reset_in14     (1'b0),                               // (terminated)
		.reset_req_in14 (1'b0),                               // (terminated)
		.reset_in15     (1'b0),                               // (terminated)
		.reset_req_in15 (1'b0)                                // (terminated)
	);

endmodule
