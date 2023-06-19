/*
 * Copyright 2019 EDGECOMPUTE Corporation.  All rights reserved.
 *
 * NOTICE TO LICENSEE:
 *
 * This source code and/or documentation ("Licensed Deliverables") are
 * subject to EDGECOMPUTE intellectual property rights under U.S. and
 * international Copyright laws.
 *
 * These Licensed Deliverables contained herein is PROPRIETARY and
 * CONFIDENTIAL to EDGECOMPUTE and is being provided under the terms and
 * conditions of a form of EDGECOMPUTE software license agreement by and
 * between EDGECOMPUTE and Licensee ("License Agreement") or electronically
 * accepted by Licensee.  Notwithstanding any terms or conditions to
 * the contrary in the License Agreement, reproduction or disclosure
 * of the Licensed Deliverables to any third party without the express
 * written consent of EDGECOMPUTE is prohibited.
 *
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, EDGECOMPUTE MAKES NO REPRESENTATION ABOUT THE
 * SUITABILITY OF THESE LICENSED DELIVERABLES FOR ANY PURPOSE.  IT IS
 * PROVIDED "AS IS" WITHOUT EXPRESS OR IMPLIED WARRANTY OF ANY KIND.
 * EDGECOMPUTE DISCLAIMS ALL WARRANTIES WITH REGARD TO THESE LICENSED
 * DELIVERABLES, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY,
 * NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE.
 * NOTWITHSTANDING ANY TERMS OR CONDITIONS TO THE CONTRARY IN THE
 * LICENSE AGREEMENT, IN NO EVENT SHALL EDGECOMPUTE BE LIABLE FOR ANY
 * SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
 * WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS
 * ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THESE LICENSED DELIVERABLES.
 *
 * U.S. Government End Users.  These Licensed Deliverables are a
 * "commercial item" as that term is defined at 48 C.F.R. 2.101 (OCT
 * 1995), consisting of "commercial computer software" and "commercial
 * computer software documentation" as such terms are used in 48
 * C.F.R. 12.212 (SEPT 1995) and is provided to the U.S. Government
 * only as a commercial end item.  Consistent with 48 C.F.R.12.212 and
 * 48 C.F.R. 227.7202-1 through 227.7202-4 (JUNE 1995), all
 * U.S. Government End Users acquire the Licensed Deliverables with
 * only those rights set forth herein.
 *
 * Any use of the Licensed Deliverables in individual and commercial
 * software must include, in the user documentation and internal
 * comments to the code, the above Disclaimer and U.S. Government End
 * Users Notice.
 */

import txu_au_pkg::au_ctl_cfg_s;
import txu_au_pkg::irf_offset_s;
import txu_au_pkg::uinstr_queue_cfg_s;
import txu_au_pkg::uau_ctl_s;
import txu_au_pkg::upwr_req_ctl_s;

module txu_uicc #(
    parameter RIP_ADDR_WIDTH = 10,
    parameter RIP_DATA_WIDTH = 32,
    parameter RIP_BEN_WIDTH  = 4,
    parameter CP_INSTR_WIDTH = 32,
    parameter UINSTR_WIDTH   = 24,

    parameter VRF_IDX_WIDTH = 7,
    parameter NUM_IRF_PORT  = 2,

    parameter MIU_TIME_WIDTH = 5,

    parameter UINSTR_CNT_WIDTH    = 32,
    parameter NUM_SCBD_ENTRY      = 64,
    parameter MIU_ARF_ENTRY_WIDTH = 4
) (
    // clock and reset
    input logic clk,
    input logic rst_n,

    // cfg target interface
    input logic        uop_phs_en,
    input au_ctl_cfg_s au_ctl_cfg,
    input irf_offset_s irf_offset,

    // uinstr queue control initiator interface
    input uinstr_queue_cfg_s uinstr_queue_cfg,
    input logic              uinstr_queue_init,
    input logic              uinstr_queue_scbd_init,

    // cp_instr target interface
    input  logic      [CP_INSTR_WIDTH - 1:0] cp_instr,
    // output logic                             cp_idle,
    output logic                             cp_dirty,
    input  logic                             cp_instr_val,
    output wire logic                        cp_instr_rdy,

    // scbd read target interface
    output logic [NUM_SCBD_ENTRY - 1:0] scbd,

    // miu scbd write target interface
    input logic [VRF_IDX_WIDTH - 1:0] miu_scbd_fe_lock_wr_idx[NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] miu_scbd_fe_lock_wr_en,
    input logic [VRF_IDX_WIDTH - 1:0] miu_scbd_unlock_wr_idx [NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] miu_scbd_unlock_wr_en,

    // x0 scbd write target interface
    input logic [VRF_IDX_WIDTH - 1:0] x0_scbd_fe_lock_wr_idx[NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] x0_scbd_fe_lock_wr_en,
    input logic [VRF_IDX_WIDTH - 1:0] x0_scbd_unlock_wr_idx [NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] x0_scbd_unlock_wr_en,

    // x1 scbd write target interface
    input logic [VRF_IDX_WIDTH - 1:0] x1_scbd_fe_lock_wr_idx[NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] x1_scbd_fe_lock_wr_en,
    input logic [VRF_IDX_WIDTH - 1:0] x1_scbd_unlock_wr_idx [NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] x1_scbd_unlock_wr_en,

    // x2 scbd write target interface
    input logic [VRF_IDX_WIDTH - 1:0] x2_scbd_fe_lock_wr_idx[NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] x2_scbd_fe_lock_wr_en,
    input logic [VRF_IDX_WIDTH - 1:0] x2_scbd_unlock_wr_idx [NUM_IRF_PORT],
    input logic [ NUM_IRF_PORT - 1:0] x2_scbd_unlock_wr_en,

    // rimm data target interface
    input  logic [RIP_DATA_WIDTH - 1:0] rimm_data,
    input  logic                        rimm_data_val,
    output logic                        rimm_data_rdy,

    // scbd read config initiator interface
    output logic [VRF_IDX_WIDTH - 1:0] scbd_vs1_idx,
    output logic                       scbd_vs1_idx_en,
    output logic [VRF_IDX_WIDTH - 1:0] scbd_vs2_idx,
    output logic                       scbd_vs2_idx_en,

    // scbd write config initiator interface
    output logic [VRF_IDX_WIDTH - 1:0] scbd_vd1_idx,
    output logic                       scbd_vd1_idx_en,
    output logic [VRF_IDX_WIDTH - 1:0] scbd_vd2_idx,
    output logic                       scbd_vd2_idx_en,

    // rip initiator0 interface
    output logic [RIP_ADDR_WIDTH - 1:0] rip_mst0_uicc_addr,
    output logic [RIP_DATA_WIDTH - 1:0] rip_mst0_uicc_wdata,
    output logic                        rip_mst0_uicc_wen,
    output logic [ RIP_BEN_WIDTH - 1:0] rip_mst0_uicc_ben,
    output logic                        rip_mst0_uicc_val,
    input  logic                        rip_mst0_uicc_rdy,

    // rip initiator1 interface
    output logic [RIP_ADDR_WIDTH - 1:0] rip_mst1_uicc_addr,
    output logic [RIP_DATA_WIDTH - 1:0] rip_mst1_uicc_wdata,
    output logic                        rip_mst1_uicc_wen,
    output logic [ RIP_BEN_WIDTH - 1:0] rip_mst1_uicc_ben,
    output logic                        rip_mst1_uicc_val,
    input  logic                        rip_mst1_uicc_rdy,

    // ctl initiator interface
    output uau_ctl_s      uau_ctl,
    output upwr_req_ctl_s upwr_req_ctl,
    output logic          miu_overlap_en,
    output logic          miu_overlap_idx_sel,

    // miu uinstr initiator if
    output logic [UINSTR_WIDTH - 1:0] miu_uinstr,
    output logic                      miu_uinstr_val,
    input  logic                      miu_uinstr_rdy,

    // miu st uinstr initiator if
    output logic [UINSTR_WIDTH - 1:0] miu_st_uinstr,
    output logic                      miu_st_uinstr_val,
    input  logic                      miu_st_uinstr_rdy,

    // x0 uinstr initiator if
    output logic [UINSTR_WIDTH - 1:0] x0_uinstr,
    output logic                      x0_uinstr_val,
    input  logic                      x0_uinstr_rdy,

    // x1 uinstr initiator if
    output logic [UINSTR_WIDTH - 1:0] x1_uinstr,
    output logic                      x1_uinstr_val,
    input  logic                      x1_uinstr_rdy,

    // x2 uinstr initiator if
    output logic [UINSTR_WIDTH - 1:0] x2_uinstr,
    output logic                      x2_uinstr_val,
    input  logic                      x2_uinstr_rdy,

    // au config initiator interface
    input  logic                          profile_start_trig,
    input  logic                          profile_end_trig,
    output logic [UINSTR_CNT_WIDTH - 1:0] uinstr_cnt,

    // uinstr queue status initiator interface
    output wire logic uinstr_queue_is_empty,
    output wire logic uinstr_queue_is_full,

    // monitor initiator interface
    output logic [RIP_DATA_WIDTH - 1:0] mon_uinstr_queue_cnt,
    output logic [RIP_DATA_WIDTH - 1:0] mon_rcp_instr,
    output logic [RIP_DATA_WIDTH - 1:0] mon_scbd0,
    output logic [RIP_DATA_WIDTH - 1:0] mon_scbd1,
    output logic [RIP_DATA_WIDTH - 1:0] mon_scbd2,
    output logic [RIP_DATA_WIDTH - 1:0] mon_scbd3,

    // Power Manager initiator interface
    input logic pwr_gnt
);

  //====================================================================
  // Local Parameters and Typedefs
  //====================================================================
  import txu_au_pkg::rcp_instr_s;
  import txu_au_pkg::imm_uinstr_s;
  import txu_au_pkg::imm_uinstr_vec_s;
  import txu_au_pkg::rimm_uinstr_s;
  import txu_au_pkg::rimm_uinstr_vec_s;
  import txu_au_pkg::scbd_src_decoder;
  import txu_au_pkg::scbd_dst_decoder;
  import txu_au_pkg::uscbd_ctl_s;
  import txu_au_pkg::uscbd_cfg0_s;
  import txu_au_pkg::uscbd_cfg1_s;
  import txu_au_pkg::update_irf_idx;
  import txu_au_pkg::update_rcp_instr;
  import txu_au_pkg::UINSTR_TYPE_WIDTH;
  import txu_au_pkg::RCP_INSTR_QUEUE_DEPTH;
  import txu_au_pkg::RCP_INSTR_QUEUE_DEPTH_WIDTH;
  import txu_au_pkg::RCP_INSTR_S_WIDTH;
  import txu_au_pkg::UINSTR_SEL_WIDTH;
  import txu_au_pkg::NUM_UINSTR_TYPE;
  import txu_au_pkg::CTL_UINSTR_TYPE;
  import txu_au_pkg::MIU_UINSTR_TYPE;
  import txu_au_pkg::UMEM_UINSTR_TYPE0;
  import txu_au_pkg::UMEM_UINSTR_TYPE1;
  import txu_au_pkg::UMEM_UINSTR_TYPE2;
  import txu_au_pkg::UMEM_UINSTR_TYPE3;
  import txu_au_pkg::UMEM_UINSTR_TYPE4;
  import txu_au_pkg::IMM_UINSTR_TYPE;
  import txu_au_pkg::X0_UINSTR_TYPE;
  import txu_au_pkg::X2_UINSTR_TYPE;
  import txu_au_pkg::CTL_UINSTR_TYPE;
  import txu_au_pkg::MIU_UINSTR_SEL;
  import txu_au_pkg::X0_UINSTR_SEL;
  import txu_au_pkg::X2_UINSTR_SEL;
  import txu_au_pkg::IMM_UINSTR_SEL;
  import txu_au_pkg::RIMM_UINSTR_SEL;
  import txu_au_pkg::CTL_UINSTR_SEL;
  import txu_au_pkg::MIU_ST_UINSTR_SEL;
  import txu_au_pkg::X1_UINSTR_SEL;
  import txu_au_pkg::USCBD_CTL_S_WIDTH;
  import txu_au_pkg::USCBD_CFG0_S_WIDTH;
  import txu_au_pkg::USCBD_CFG1_S_WIDTH;
  import txu_au_pkg::UAU_CTL_S_WIDTH;
  import txu_au_pkg::UPWR_REQ_CTL_S_WIDTH;
  import txu_au_pkg::IRF_IDX_WIDTH;
  import txu_au_pkg::IMM_UINSTR_S_WIDTH;
  import txu_au_pkg::RIMM_UINSTR_S_WIDTH;

  import txu_au_common_pkg::UOP_PHS_WIDTH;
  import txu_au_common_pkg::PORT0;
  import txu_au_common_pkg::PORT1;

  // CTL
  localparam UHOLD_TYPE = 2'd0;
  localparam URIMM_TYPE = 2'd1;
  localparam UCTL_A_TYPE = 2'd2;
  localparam UCTL_B_TYPE = 2'd3;

  // UCTL_A
  localparam USCBD_CTL = 3'd0;
  localparam USCBD_CFG0 = 3'd1;
  localparam USCBD_CFG1 = 3'd2;
  localparam UINSTR_GRP_CTL = 3'd3;
  localparam USET_QUEUE = 3'd4;
  localparam UWAIT_QUEUE = 3'd5;

  // UCTL_B
  import txu_au_pkg::UAU_CTL;
  import txu_au_pkg::UPWR_REQ_CTL;

  localparam REG_INSTR_SEL = 3'd0;
  localparam USCBD_CTL_SEL = 3'd1;
  localparam USCBD_CFG0_SEL = 3'd2;
  localparam USCBD_CFG1_SEL = 3'd3;
  localparam UINSTR_GRP_CTL_SEL = 3'd4;

  localparam NO_BYPASS = 0;
  localparam BYPASS = 1;
  localparam SIG_WIDTH = RCP_INSTR_S_WIDTH + (VRF_IDX_WIDTH * 4) + 4 + 3;

  localparam RIMM_CTL_MODE1 = 1;
  localparam RIMM_CTL_MODE2 = 2;

  //====================================================================
  // Signals
  //====================================================================
  logic [UOP_PHS_WIDTH - 1:0] uop_phs;
  logic is_uop_phs0;
  logic is_uop_phs1;
  logic is_uop_phs2;
  logic is_uop_phs3;

  logic uinstr_cnt_en;

  logic [1:0] fif0_rcp_instr_sel;
  logic [(RCP_INSTR_QUEUE_DEPTH_WIDTH + 1) - 1:0] fif0_dn_uwait_queue_num_item;
  logic [3:0] fif0_rcp_instr_val_vec;
  logic [3:0] fif0_rcp_instr_rdy_vec;

  rcp_instr_s fif0_rcp_instr;
  rcp_instr_s fif0_dn_rcp_instr;
  logic fif0_dn_rcp_instr_val;
  logic fif0_dn_rcp_instr_rdy;
  logic fif0_dn_uinstr_queue_is_empty;
  logic fif0_dn_uinstr_queue_is_full;
  logic [(RCP_INSTR_QUEUE_DEPTH_WIDTH + 1) - 1:0] fif0_dn_uinstr_queue_num_item;

  logic [2:0] fif1_rcp_instr_sel;
  logic [7:0] fif1_rcp_instr_val_vec;
  logic [7:0] fif1_rcp_instr_rdy_vec;
  uscbd_ctl_s fif1_dn_uscbd_ctl;
  uscbd_cfg0_s fif1_dn_uscbd_cfg0;
  uscbd_cfg1_s fif1_dn_uscbd_cfg1;
  logic [(RCP_INSTR_QUEUE_DEPTH_WIDTH + 1) - 1:0] fif1_dn_uinstr_grp_ctl;
  logic fif1_dn_rcp_instr_en;
  rcp_instr_s fif1_dn_rcp_instr;
  logic fif1_dn_rcp_instr_val;
  logic fif1_dn_rcp_instr_rdy;
  uscbd_ctl_s fif1_dn_d_uscbd_ctl;
  uscbd_cfg0_s fif1_dn_d_uscbd_cfg0;
  uscbd_cfg1_s fif1_dn_d_uscbd_cfg1;

  logic [IRF_IDX_WIDTH - 1:0] p0_vs1_idx_out;
  logic [IRF_IDX_WIDTH - 1:0] p0_vs2_idx_out;
  logic [IRF_IDX_WIDTH - 1:0] p0_vd1_idx_out;
  rcp_instr_s p0_dn_rcp_instr;
  logic p0_dn_rcp_instr_val;
  logic p0_dn_rcp_instr_rdy;
  uscbd_ctl_s p0_dn_uscbd_ctl;
  uscbd_cfg0_s p0_dn_uscbd_cfg0;
  uscbd_cfg1_s p0_dn_uscbd_cfg1;
  logic [IRF_IDX_WIDTH - 1:0] p0_dn_vs1_idx_out;
  logic [IRF_IDX_WIDTH - 1:0] p0_dn_vs2_idx_out;
  logic [IRF_IDX_WIDTH - 1:0] p0_dn_vd1_idx_out;
  logic p0_dn_do_update_instr;

  rcp_instr_s p1_rcp_instr;
  rcp_instr_s p1_dn_rcp_instr;
  uscbd_ctl_s p1_dn_uscbd_ctl;
  uscbd_cfg0_s p1_dn_uscbd_cfg0;
  uscbd_cfg1_s p1_dn_uscbd_cfg1;
  logic p1_dn_rcp_instr_val;
  logic p1_dn_rcp_instr_rdy;

  logic [VRF_IDX_WIDTH - 1:0] fif2_scbd_vs1_idx;
  logic fif2_scbd_vs1_idx_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_scbd_vs2_idx;
  logic fif2_scbd_vs2_idx_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_scbd_vd1_idx;
  logic fif2_scbd_vd1_idx_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_scbd_vd2_idx;
  logic fif2_scbd_vd2_idx_en;
  logic fif2_miu_if0_up_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_miu_scbd_lock_wr_idx_loc[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_miu_scbd_lock_wr_en_loc;
  logic [VRF_IDX_WIDTH - 1:0] fif2_miu_scbd_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_miu_scbd_lock_wr_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_miu_scbd_fe_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_miu_scbd_fe_lock_wr_en;
  logic fif2_x0_if0_up_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x0_scbd_lock_wr_idx_loc[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x0_scbd_lock_wr_en_loc;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x0_scbd_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x0_scbd_lock_wr_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x0_scbd_fe_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x0_scbd_fe_lock_wr_en;
  logic fif2_x1_if0_up_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x1_scbd_lock_wr_idx_loc[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x1_scbd_lock_wr_en_loc;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x1_scbd_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x1_scbd_lock_wr_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x1_scbd_fe_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x1_scbd_fe_lock_wr_en;
  logic fif2_x2_if0_up_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x2_scbd_lock_wr_idx_loc[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x2_scbd_lock_wr_en_loc;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x2_scbd_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x2_scbd_lock_wr_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_x2_scbd_fe_lock_wr_idx[NUM_IRF_PORT];
  logic [NUM_IRF_PORT - 1:0] fif2_x2_scbd_fe_lock_wr_en;
  logic fif2_is_ust;
  rcp_instr_s fif2_dn_rcp_instr;
  logic [VRF_IDX_WIDTH - 1:0] fif2_dn_scbd_vs1_idx;
  logic fif2_dn_scbd_vs1_idx_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_dn_scbd_vs2_idx;
  logic fif2_dn_scbd_vs2_idx_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_dn_scbd_vd1_idx;
  logic fif2_dn_scbd_vd1_idx_en;
  logic [VRF_IDX_WIDTH - 1:0] fif2_dn_scbd_vd2_idx;
  logic fif2_dn_scbd_vd2_idx_en;
  logic fif2_dn_miu_overlap_en;
  logic fif2_dn_miu_overlap_idx_sel;
  logic fif2_dn_rcp_instr_val;
  logic fif2_dn_rcp_instr_rdy;
  logic fif2_dn_is_ust;

  logic [UINSTR_SEL_WIDTH - 1:0] fif3_uinstr_sel;
  logic [NUM_SCBD_ENTRY - 1:0] fif3_scbd;
  logic fif3_scbd_vs1_lock;
  logic fif3_scbd_vs2_lock;
  logic fif3_scbd_vs_lock;
  logic fif3_miu_st_uinstr_val;
  logic fif3_miu_st_uinstr_rdy;
  logic fif3_ctl_uinstr_val;
  logic fif3_ctl_uinstr_rdy_loc;
  logic fif3_ctl_uinstr_rdy;
  logic fif3_rimm_uinstr_val;
  logic fif3_rimm_uinstr_rdy;
  logic fif3_imm_uinstr_val;
  logic fif3_imm_uinstr_rdy;
  logic fif3_x2_uinstr_val;
  logic fif3_x2_uinstr_rdy;
  logic fif3_x1_uinstr_val;
  logic fif3_x1_uinstr_rdy;
  logic fif3_x0_uinstr_val;
  logic fif3_x0_uinstr_rdy;
  logic fif3_miu_uinstr_val;
  logic fif3_miu_uinstr_rdy;
  imm_uinstr_s fif3_imm_uinstr;
  logic fif3_rimm_uinstr_cmb_val;
  logic fif3_rimm_uinstr_cmb_rdy;
  rimm_uinstr_s fif3_rimm_uinstr;
  logic [NUM_UINSTR_TYPE - 1:0] fif3_uinstr_val_vec;
  logic [NUM_UINSTR_TYPE - 1:0] fif3_uinstr_rdy_vec;
  logic fif3_miu_uinstr_rdy_loc;
  logic fif3_miu_st_uinstr_rdy_loc;
  logic fif3_x1_uinstr_rdy_loc;
  logic fif3_x0_uinstr_rdy_loc;
  logic fif3_x2_uinstr_rdy_loc;

  logic fif3_dn_imm_uinstr_vec_val;
  logic fif3_dn_imm_uinstr_vec_rdy;
  imm_uinstr_vec_s fif3_dn_imm_uinstr_vec;
  rimm_uinstr_vec_s fif3_dn_rimm_uinstr_vec;
  logic fif3_dn_rimm_uinstr_vec_val;
  logic fif3_dn_rimm_uinstr_vec_rdy;
  uau_ctl_s fif3_dn_uau_ctl;
  upwr_req_ctl_s fif3_dn_upwr_req_ctl;

  logic pwr_gnt_loc;
  logic pwr_gnt_loc1;

  //====================================================================
  // Functions
  //====================================================================

  //====================================================================
  // Interfaces
  //====================================================================

  //====================================================================
  // Operation
  //====================================================================
  //--------------------------------------------
  // uop_phs
  //--------------------------------------------
  always_ff @(posedge clk, negedge rst_n) begin : ff_uop_phs
    if (!rst_n) begin
      uop_phs <= '0;
    end else if (uop_phs_en) begin
      uop_phs <= UOP_PHS_WIDTH'(uop_phs + UOP_PHS_WIDTH'(1'b1));
    end
  end : ff_uop_phs

  // is_uop_phs[0-3]
  assign is_uop_phs0    = (uop_phs == txu_au_common_pkg::UOP_PHS0);
  assign is_uop_phs1    = (uop_phs == txu_au_common_pkg::UOP_PHS1);
  assign is_uop_phs2    = (uop_phs == txu_au_common_pkg::UOP_PHS2);
  assign is_uop_phs3    = (uop_phs == txu_au_common_pkg::UOP_PHS3);

  //--------------------------------------------
  // FIF0 stage
  //--------------------------------------------
  // fif0_rcp_instr
  assign fif0_rcp_instr = rcp_instr_s'(cp_instr[(CP_INSTR_WIDTH-1)-:RCP_INSTR_S_WIDTH]);

  // fif0_rcp_instr_sel
  always_comb begin : comb_fif0_rcp_instr_sel
    fif0_rcp_instr_sel = 1'b0;
    if ((fif0_rcp_instr.uinstr_type == CTL_UINSTR_TYPE) &&
      (fif0_rcp_instr.vr_enc.ctl.enc2 == UCTL_A_TYPE)) begin
      case (fif0_rcp_instr.vr_enc.ctl.enc1)
        USET_QUEUE: begin
          fif0_rcp_instr_sel = 2'd1;
        end
        UWAIT_QUEUE: begin
          fif0_rcp_instr_sel = 2'd2;
        end
        default: begin
          fif0_rcp_instr_sel = 2'd0;
        end
      endcase
    end
  end : comb_fif0_rcp_instr_sel

  // rcp_instr p2p_demux4_ctl
  p2p_if_demux4_ctl u_rcp_instr_p2p_demux4_ctl (
      // p2p target interface
      .up_vld  (cp_instr_val),
      .up_rdy  (cp_instr_rdy),
      .port_sel(fif0_rcp_instr_sel),

      // p2p initiator interface
      .dn_vld_vec(fif0_rcp_instr_val_vec),
      .dn_rdy_vec(fif0_rcp_instr_rdy_vec)
  );  // p2p_if_demux2_ctl u_rcp_instr_p2p_demux2_ctl

  assign fif0_rcp_instr_rdy_vec[1] = 1'b1;
  assign fif0_rcp_instr_rdy_vec[3] = 1'b1;

  // uwait_queue num_item
  always_ff @(posedge clk, negedge rst_n) begin : ff_fif0_dn_uwait_queue_num_item
    if (!rst_n) begin
      fif0_dn_uwait_queue_num_item <= (RCP_INSTR_QUEUE_DEPTH_WIDTH + 1)'(7'd64);
    end else if (fif0_rcp_instr_val_vec[1]) begin
      fif0_dn_uwait_queue_num_item <=
        fif0_rcp_instr.vr_enc.ctl.params[(RCP_INSTR_QUEUE_DEPTH_WIDTH + 1) - 1:0];
    end
  end : ff_fif0_dn_uwait_queue_num_item

  // fif0_rcp_instr_rdy_vec[2]
  always_comb begin : comb_uwait_queue
    if (fif0_dn_uinstr_queue_num_item <= fif0_dn_uwait_queue_num_item) begin
      fif0_rcp_instr_rdy_vec[2] = 1'b1;
    end else begin
      fif0_rcp_instr_rdy_vec[2] = 1'b0;
    end
  end : comb_uwait_queue

  // cp_instr queue
  p2p_if_vfifo #(
      .DATA_WIDTH(RCP_INSTR_S_WIDTH),
      .MAX_DEPTH (RCP_INSTR_QUEUE_DEPTH),
      .IDX_WIDTH (RCP_INSTR_QUEUE_DEPTH_WIDTH)
  ) u_p2p_if_vfifo (
      // clock and reset
      .clk,
      .rst_n,

      // initialization target interface
      .init(uinstr_queue_init),

      // config target interface
      .depth(uinstr_queue_cfg.depth),

      // p2p target interface
      .up_data(fif0_rcp_instr),
      .up_vld (fif0_rcp_instr_val_vec[0]),
      .up_rdy (fif0_rcp_instr_rdy_vec[0]),

      // p2p initiator interface
      .dn_data(fif0_dn_rcp_instr),
      .dn_vld (fif0_dn_rcp_instr_val),
      .dn_rdy (fif0_dn_rcp_instr_rdy),

      // fifo status interface
      .is_empty(fif0_dn_uinstr_queue_is_empty),
      .is_full (fif0_dn_uinstr_queue_is_full),
      .num_item(fif0_dn_uinstr_queue_num_item)
  );  // p2p_if_vfifo u_p2p_if_vfifo

  //--------------------------------------------
  // FIF1 stage
  //--------------------------------------------
  // fif1_rcp_instr_sel
  always_comb begin : comb_fif1_rcp_instr_sel
    fif1_rcp_instr_sel = REG_INSTR_SEL;
    if ((fif0_dn_rcp_instr.uinstr_type == CTL_UINSTR_TYPE) &&
      (fif0_dn_rcp_instr.vr_enc.ctl.enc2 == UCTL_A_TYPE)) begin
      case (fif0_dn_rcp_instr.vr_enc.ctl.enc1)
        USCBD_CTL: begin
          fif1_rcp_instr_sel = USCBD_CTL_SEL;
        end
        USCBD_CFG0: begin
          fif1_rcp_instr_sel = USCBD_CFG0_SEL;
        end
        USCBD_CFG1: begin
          fif1_rcp_instr_sel = USCBD_CFG1_SEL;
        end
        UINSTR_GRP_CTL: begin
          fif1_rcp_instr_sel = UINSTR_GRP_CTL_SEL;
        end
        default: begin
          fif1_rcp_instr_sel = USCBD_CTL_SEL;
        end
      endcase
    end
  end : comb_fif1_rcp_instr_sel

  // au_ctl demux
  p2p_if_demux8_ctl u0_p2p_demux8_ctl (
      // p2p target interface
      .up_vld  (fif0_dn_rcp_instr_val),
      .up_rdy  (fif0_dn_rcp_instr_rdy),
      .port_sel(fif1_rcp_instr_sel),

      // p2p initiator interface
      .dn_vld_vec(fif1_rcp_instr_val_vec),
      .dn_rdy_vec(fif1_rcp_instr_rdy_vec)
  );  // p2p_if_demux8_ctl u0_p2p_demux8_ctl

  // fif1_dn_uscbd_ctl
  assign fif1_rcp_instr_rdy_vec[USCBD_CTL_SEL] = 1'b1;
  always_ff @(posedge clk, negedge rst_n) begin : ff_uscbd_ctl
    if (!rst_n) begin
      fif1_dn_uscbd_ctl <= USCBD_CTL_S_WIDTH'(15'h1FFF);
    end else if (fif1_rcp_instr_val_vec[USCBD_CTL_SEL]) begin
      fif1_dn_uscbd_ctl <= fif0_dn_rcp_instr[USCBD_CTL_S_WIDTH-1:0];
    end
  end : ff_uscbd_ctl

  // fif1_dn_uscbd_cfg0
  assign fif1_rcp_instr_rdy_vec[USCBD_CFG0_SEL] = 1'b1;
  always_ff @(posedge clk, negedge rst_n) begin : ff_uscbd_cfg0
    if (!rst_n) begin
      fif1_dn_uscbd_cfg0 <= USCBD_CFG0_S_WIDTH'(16'h0000);
    end else if (fif1_rcp_instr_val_vec[USCBD_CFG0_SEL]) begin
      fif1_dn_uscbd_cfg0 <= fif0_dn_rcp_instr[USCBD_CFG0_S_WIDTH-1:0];
    end
  end : ff_uscbd_cfg0

  // fif1_dn_uscbd_cfg1
  assign fif1_rcp_instr_rdy_vec[USCBD_CFG1_SEL] = 1'b1;
  always_ff @(posedge clk, negedge rst_n) begin : ff_uscbd_cfg1
    if (!rst_n) begin
      fif1_dn_uscbd_cfg1 <= USCBD_CFG1_S_WIDTH'(16'h0000);
    end else if (fif1_rcp_instr_val_vec[USCBD_CFG1_SEL]) begin
      fif1_dn_uscbd_cfg1 <= fif0_dn_rcp_instr[USCBD_CFG1_S_WIDTH-1:0];
    end
  end : ff_uscbd_cfg1

  // fif1_dn_uinstr_grp_ctl
  assign fif1_rcp_instr_rdy_vec[UINSTR_GRP_CTL_SEL] = 1'b1;
  always_ff @(posedge clk, negedge rst_n) begin : ff_uinstr_grp_ctl
    if (!rst_n) begin
      fif1_dn_uinstr_grp_ctl <= (RCP_INSTR_QUEUE_DEPTH_WIDTH + 1)'(7'h00);
    end else if (fif1_rcp_instr_val_vec[UINSTR_GRP_CTL_SEL]) begin
      fif1_dn_uinstr_grp_ctl <= fif0_dn_rcp_instr[(RCP_INSTR_QUEUE_DEPTH_WIDTH+1)-1:0];
    end
  end : ff_uinstr_grp_ctl

  // fif1_dn_rcp_instr_en
  assign fif1_dn_rcp_instr_en        = (fif0_dn_uinstr_queue_num_item >= fif1_dn_uinstr_grp_ctl);

  // unused ports
  assign fif1_rcp_instr_rdy_vec[7:5] = 3'b111;

  // fif1_dn_rcp_instr
  p2p_if_pipe_en_init #(
      .DATA_WIDTH(RCP_INSTR_S_WIDTH + USCBD_CTL_S_WIDTH + USCBD_CFG0_S_WIDTH + USCBD_CFG1_S_WIDTH)
  ) u_p2p_if_pipe (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_init),

      // enable target interface
      .en(fif1_dn_rcp_instr_en),

      // p2p target interface
      .up_data({fif0_dn_rcp_instr, fif1_dn_uscbd_ctl, fif1_dn_uscbd_cfg0, fif1_dn_uscbd_cfg1}),

      .up_vld(fif1_rcp_instr_val_vec[REG_INSTR_SEL]),
      .up_rdy(fif1_rcp_instr_rdy_vec[REG_INSTR_SEL]),

      // p2p initiator interface
      .dn_data({
        fif1_dn_rcp_instr, fif1_dn_d_uscbd_ctl, fif1_dn_d_uscbd_cfg0, fif1_dn_d_uscbd_cfg1
      }),

      .dn_vld(fif1_dn_rcp_instr_val),
      .dn_rdy(fif1_dn_rcp_instr_rdy)
  );  // p2p_if_pipe_init u_p2p_if_pipe

  //--------------------------------------------
  // P0 stage
  //--------------------------------------------
  always_comb begin : comb_idx_out
    update_irf_idx(.rcp_instr(fif1_dn_rcp_instr), .vs1_offset(irf_offset.vs1_offset),
                   .vs2_offset(irf_offset.vs2_offset), .vd1_offset(irf_offset.vd1_offset),
                   .vs1_idx_out(p0_vs1_idx_out), .vs2_idx_out(p0_vs2_idx_out),
                   .vd1_idx_out(p0_vd1_idx_out));
  end : comb_idx_out

  // p0_dn_rcp_instr
  p2p_if_pipe_init #(
      .DATA_WIDTH(RCP_INSTR_S_WIDTH + USCBD_CTL_S_WIDTH + USCBD_CFG0_S_WIDTH + USCBD_CFG1_S_WIDTH + 3*IRF_IDX_WIDTH + 1)
  ) u_p0_p2p_if_pipe (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_init),

      // p2p target interface
      .up_data({
        fif1_dn_rcp_instr,
        fif1_dn_d_uscbd_ctl,
        fif1_dn_d_uscbd_cfg0,
        fif1_dn_d_uscbd_cfg1,
        p0_vs1_idx_out,
        p0_vs2_idx_out,
        p0_vd1_idx_out,
        irf_offset.do_update_instr
      }),

      .up_vld(fif1_dn_rcp_instr_val),
      .up_rdy(fif1_dn_rcp_instr_rdy),

      // p2p initiator interface
      .dn_data({
        p0_dn_rcp_instr,
        p0_dn_uscbd_ctl,
        p0_dn_uscbd_cfg0,
        p0_dn_uscbd_cfg1,
        p0_dn_vs1_idx_out,
        p0_dn_vs2_idx_out,
        p0_dn_vd1_idx_out,
        p0_dn_do_update_instr
      }),

      .dn_vld(p0_dn_rcp_instr_val),
      .dn_rdy(p0_dn_rcp_instr_rdy)
  );  // p2p_if_pipe_init u_p0_p2p_if_pipe

  //--------------------------------------------
  // P1 stage
  //--------------------------------------------
  // p1_rcp_instr
  assign p1_rcp_instr = (p0_dn_do_update_instr) ? update_rcp_instr(
      .rcp_instr(p0_dn_rcp_instr),
      .vs1_idx(p0_dn_vs1_idx_out),
      .vs2_idx(p0_dn_vs2_idx_out),
      .vd1_idx(p0_dn_vd1_idx_out)
  ) : p0_dn_rcp_instr;

  // p1_dn_rcp_instr
  p2p_if_pipe_init #(
      .DATA_WIDTH(RCP_INSTR_S_WIDTH + USCBD_CTL_S_WIDTH + USCBD_CFG0_S_WIDTH + USCBD_CFG1_S_WIDTH)
  ) u_p1_p2p_if_pipe (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_init),

      // p2p target interface
      .up_data({p1_rcp_instr, p0_dn_uscbd_ctl, p0_dn_uscbd_cfg0, p0_dn_uscbd_cfg1}),

      .up_vld(p0_dn_rcp_instr_val),
      .up_rdy(p0_dn_rcp_instr_rdy),

      // p2p initiator interface
      .dn_data({p1_dn_rcp_instr, p1_dn_uscbd_ctl, p1_dn_uscbd_cfg0, p1_dn_uscbd_cfg1}),

      .dn_vld(p1_dn_rcp_instr_val),
      .dn_rdy(p1_dn_rcp_instr_rdy)
  );  // p2p_if_pipe_init u_p1_p2p_if_pipe

  //--------------------------------------------
  // FIF2 stage
  //--------------------------------------------
  always_comb begin : comb_scbd_src_decoder
    // vs[12]_idx, vs[12]_idx_en
    scbd_src_decoder(
        .rcp_instr(p1_dn_rcp_instr), .miu_scbd_vs1_en(p1_dn_uscbd_ctl.miu_scbd_vs1_en),
        .x0_scbd_vs1_en(p1_dn_uscbd_ctl.x0_scbd_vs1_en),
        .x0_scbd_vs2_en(p1_dn_uscbd_ctl.x0_scbd_vs2_en),
        .x1_scbd_vs1_en(p1_dn_uscbd_ctl.x1_scbd_vs1_en),
        .x1_scbd_vs2_en(p1_dn_uscbd_ctl.x1_scbd_vs2_en),
        .x2_scbd_vs1_en(p1_dn_uscbd_ctl.x2_scbd_vs1_en),
        .x2_scbd_vs2_en(p1_dn_uscbd_ctl.x2_scbd_vs2_en), .miu_vs1_cfg(p1_dn_uscbd_cfg0.miu_vs1_cfg),
        .x0_vs1_cfg(p1_dn_uscbd_cfg0.x0_vs1_cfg), .x0_vs2_cfg(p1_dn_uscbd_cfg0.x0_vs2_cfg),
        .x1_vs1_cfg(p1_dn_uscbd_cfg1.x1_vs1_cfg), .x1_vs2_cfg(p1_dn_uscbd_cfg1.x1_vs2_cfg),
        .x2_vs1_cfg(p1_dn_uscbd_cfg1.x2_vs1_cfg), .x2_vs2_cfg(p1_dn_uscbd_cfg1.x2_vs2_cfg),

        .vs1_idx(fif2_scbd_vs1_idx), .vs2_idx(fif2_scbd_vs2_idx), .vs1_idx_en(fif2_scbd_vs1_idx_en),
        .vs2_idx_en(fif2_scbd_vs2_idx_en), .is_ust(fif2_is_ust));
  end : comb_scbd_src_decoder

  always_comb begin : comb_scbd_dst_decoder
    // vd[12]_idx, vd[12]_idx_en
    {fif2_scbd_vd1_idx, fif2_scbd_vd1_idx_en,
     fif2_scbd_vd2_idx, fif2_scbd_vd2_idx_en} =
    scbd_dst_decoder(
      .rcp_instr(p1_dn_rcp_instr),
      .miu_scbd_vd1_lock_en(p1_dn_uscbd_ctl.miu_scbd_vd1_lock_en),
      .x0_scbd_vd1_lock_en(p1_dn_uscbd_ctl.x0_scbd_vd1_lock_en),
      .x0_scbd_vd2_lock_en(p1_dn_uscbd_ctl.x0_scbd_vd2_lock_en),
      .x1_scbd_vd1_lock_en(p1_dn_uscbd_ctl.x1_scbd_vd1_lock_en),
      .x2_scbd_vd1_lock_en(p1_dn_uscbd_ctl.x2_scbd_vd1_lock_en),
      .x2_scbd_vd2_lock_en(p1_dn_uscbd_ctl.x2_scbd_vd2_lock_en),
      .miu_vd1_cfg(p1_dn_uscbd_cfg0.miu_vd1_cfg),
      .x0_vd1_cfg(p1_dn_uscbd_cfg0.x0_vd1_cfg),
      .x0_vd2_cfg(p1_dn_uscbd_cfg0.x0_vd2_cfg),
      .x1_vd1_cfg(p1_dn_uscbd_cfg1.x1_vd1_cfg),
      .x2_vd1_cfg(p1_dn_uscbd_cfg1.x2_vd1_cfg),
      .x2_vd2_cfg(p1_dn_uscbd_cfg1.x2_vd2_cfg)
    );
  end : comb_scbd_dst_decoder

  // fif2_dn_rcp_instr
  p2p_if_pipe_init #(
      .DATA_WIDTH(SIG_WIDTH)
  ) u_p2p_if_pipe1 (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_init),

      // p2p target interface
      .up_data({
        p1_dn_rcp_instr,
        fif2_scbd_vs1_idx,
        fif2_scbd_vs1_idx_en,
        fif2_scbd_vs2_idx,
        fif2_scbd_vs2_idx_en,
        fif2_scbd_vd1_idx,
        fif2_scbd_vd1_idx_en,
        fif2_scbd_vd2_idx,
        fif2_scbd_vd2_idx_en,
        fif2_is_ust,
        p1_dn_uscbd_ctl.miu_overlap_en,
        p1_dn_uscbd_ctl.miu_overlap_idx_sel
      }),

      .up_vld(p1_dn_rcp_instr_val),
      .up_rdy(p1_dn_rcp_instr_rdy),

      // p2p initiator interface
      .dn_data({
        fif2_dn_rcp_instr,
        fif2_dn_scbd_vs1_idx,
        fif2_dn_scbd_vs1_idx_en,
        fif2_dn_scbd_vs2_idx,
        fif2_dn_scbd_vs2_idx_en,
        fif2_dn_scbd_vd1_idx,
        fif2_dn_scbd_vd1_idx_en,
        fif2_dn_scbd_vd2_idx,
        fif2_dn_scbd_vd2_idx_en,
        fif2_dn_is_ust,
        fif2_dn_miu_overlap_en,
        fif2_dn_miu_overlap_idx_sel
      }),

      .dn_vld(fif2_dn_rcp_instr_val),
      .dn_rdy(fif2_dn_rcp_instr_rdy)
  );  // p2p_if_pipe_init u_p2p_if_pipe

  //--------------------------------------------
  // FIF3 stage
  //--------------------------------------------
  // fif3_uinstr_sel
  always_comb begin : comb_fif3_uinstr_sel
    case (fif2_dn_rcp_instr.uinstr_type)
      MIU_UINSTR_TYPE,
      UMEM_UINSTR_TYPE0,
      UMEM_UINSTR_TYPE1,
      UMEM_UINSTR_TYPE2,
      UMEM_UINSTR_TYPE3,
      UMEM_UINSTR_TYPE4:
      begin
        if (fif2_dn_miu_overlap_en && fif2_dn_is_ust) begin
          fif3_uinstr_sel = MIU_ST_UINSTR_SEL;
        end else begin
          fif3_uinstr_sel = MIU_UINSTR_SEL;
        end
      end
      X0_UINSTR_TYPE: begin
        fif3_uinstr_sel = X0_UINSTR_SEL;
      end
      X2_UINSTR_TYPE: begin
        fif3_uinstr_sel = X2_UINSTR_SEL;
      end
      IMM_UINSTR_TYPE: begin
        fif3_uinstr_sel = IMM_UINSTR_SEL;
      end
      CTL_UINSTR_TYPE: begin
        case (fif2_dn_rcp_instr.vr_enc.ctl.enc2)
          UHOLD_TYPE: begin
            fif3_uinstr_sel = CTL_UINSTR_SEL;
          end
          URIMM_TYPE: begin
            fif3_uinstr_sel = RIMM_UINSTR_SEL;
          end
          UCTL_B_TYPE: begin
            fif3_uinstr_sel = CTL_UINSTR_SEL;
          end
          default: begin
            fif3_uinstr_sel = CTL_UINSTR_SEL;
          end
        endcase
      end
      default: // X1_UINSTR_TYPE
      begin
        fif3_uinstr_sel = X1_UINSTR_SEL;
      end
    endcase
  end : comb_fif3_uinstr_sel

  // ctl/imm/miu/x0/x1/x2 uinstr demux
  p2p_if_demux8_ctl u1_p2p_demux8_ctl (
      // p2p target interface
      .up_vld  (fif2_dn_rcp_instr_val),
      .up_rdy  (fif2_dn_rcp_instr_rdy),
      .port_sel(fif3_uinstr_sel),

      // p2p initiator interface
      .dn_vld_vec(fif3_uinstr_val_vec),
      .dn_rdy_vec(fif3_uinstr_rdy_vec)
  );  // p2p_if_demux8_ctl u1_p2p_demux8_ctl

  //--------------------------------------------
  // fif3_scbd
  //--------------------------------------------
  txu_scbd #(
      .NUM_VRF_ENTRY(NUM_SCBD_ENTRY),
      .VRF_IDX_WIDTH(VRF_IDX_WIDTH),
      .NUM_IRF_PORT (NUM_IRF_PORT)
  ) u_txu_scbd (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_scbd_init),

      // miu scbd write target interface
      .miu_scbd_lock_wr_idx(fif2_miu_scbd_lock_wr_idx),
      .miu_scbd_lock_wr_en (fif2_miu_scbd_lock_wr_en),
      .miu_scbd_unlock_wr_idx,
      .miu_scbd_unlock_wr_en,

      // x0 scbd write target interface
      .x0_scbd_lock_wr_idx(fif2_x0_scbd_lock_wr_idx),
      .x0_scbd_lock_wr_en (fif2_x0_scbd_lock_wr_en),
      .x0_scbd_unlock_wr_idx,
      .x0_scbd_unlock_wr_en,

      // x1 scbd write target interface
      .x1_scbd_lock_wr_idx(fif2_x1_scbd_lock_wr_idx),
      .x1_scbd_lock_wr_en (fif2_x1_scbd_lock_wr_en),
      .x1_scbd_unlock_wr_idx,
      .x1_scbd_unlock_wr_en,

      // x2 scbd write target interface
      .x2_scbd_lock_wr_idx(fif2_x2_scbd_lock_wr_idx),
      .x2_scbd_lock_wr_en (fif2_x2_scbd_lock_wr_en),
      .x2_scbd_unlock_wr_idx,
      .x2_scbd_unlock_wr_en,

      // read initiator interface
      .scbd(fif3_scbd)
  );  // txu_scbd u_txu_scbd

  //--------------------------------------------
  // txu_scbd_vs_rdys
  //--------------------------------------------
  assign fif3_scbd_vs1_lock = fif2_dn_scbd_vs1_idx_en & fif3_scbd[fif2_dn_scbd_vs1_idx];
  assign fif3_scbd_vs2_lock = fif2_dn_scbd_vs2_idx_en & fif3_scbd[fif2_dn_scbd_vs2_idx];
  assign fif3_scbd_vs_lock = fif3_scbd_vs1_lock | fif3_scbd_vs2_lock;

  //--------------------------------------------
  // uinstr_vals
  //--------------------------------------------
  assign {fif3_miu_st_uinstr_val, fif3_ctl_uinstr_val, fif3_rimm_uinstr_val, fif3_imm_uinstr_val,
    fif3_x2_uinstr_val, fif3_x1_uinstr_val, fif3_x0_uinstr_val, fif3_miu_uinstr_val}
    = fif3_uinstr_val_vec;

  //--------------------------------------------
  //uinstr_rdys
  //--------------------------------------------
  assign fif3_miu_uinstr_rdy_loc = miu_uinstr_rdy & (x1_uinstr_rdy | fif3_dn_uau_ctl.miu_x1_parallel_exec_en);
  assign fif3_miu_st_uinstr_rdy_loc = miu_st_uinstr_rdy & (x1_uinstr_rdy | fif3_dn_uau_ctl.miu_x1_parallel_exec_en);
  assign fif3_x1_uinstr_rdy_loc = x1_uinstr_rdy & (miu_uinstr_rdy | fif3_dn_uau_ctl.miu_x1_parallel_exec_en) &
    (miu_st_uinstr_rdy | fif3_dn_uau_ctl.miu_x1_parallel_exec_en);
  assign fif3_x0_uinstr_rdy_loc = x0_uinstr_rdy & (x2_uinstr_rdy | fif3_dn_uau_ctl.x0_x2_parallel_exec_en);
  assign fif3_x2_uinstr_rdy_loc = x2_uinstr_rdy & (x0_uinstr_rdy | fif3_dn_uau_ctl.x0_x2_parallel_exec_en);

  assign fif3_miu_uinstr_rdy = fif3_miu_uinstr_rdy_loc & is_uop_phs1 & (~fif3_scbd_vs1_lock);
  assign fif3_miu_st_uinstr_rdy = fif3_miu_st_uinstr_rdy_loc & is_uop_phs0 & (~fif3_scbd_vs1_lock);
  assign fif3_x0_uinstr_rdy = fif3_x0_uinstr_rdy_loc & is_uop_phs3 & (~fif3_scbd_vs_lock);
  assign fif3_x1_uinstr_rdy = fif3_x1_uinstr_rdy_loc & is_uop_phs1 & (~fif3_scbd_vs_lock);
  assign fif3_x2_uinstr_rdy = fif3_x2_uinstr_rdy_loc & is_uop_phs3 & (~fif3_scbd_vs_lock);

  assign fif3_ctl_uinstr_rdy_loc = (~fif3_scbd_vs_lock) & au_ctl_cfg.go;
  assign pwr_gnt_loc = (uinstr_queue_cfg.pwr_gnt_mode) ? uinstr_queue_cfg.manual_pwr_gnt : pwr_gnt;
  assign pwr_gnt_loc1 = fif3_dn_upwr_req_ctl.inv_pwr_gnt ^ pwr_gnt_loc;

  assign fif3_ctl_uinstr_rdy = fif3_ctl_uinstr_rdy_loc & (pwr_gnt_loc1 | fif3_dn_upwr_req_ctl.override_pwr_gnt);

  assign fif3_uinstr_rdy_vec = {
    fif3_miu_st_uinstr_rdy,
    fif3_ctl_uinstr_rdy,
    fif3_rimm_uinstr_rdy,
    fif3_imm_uinstr_rdy,
    fif3_x2_uinstr_rdy,
    fif3_x1_uinstr_rdy,
    fif3_x0_uinstr_rdy,
    fif3_miu_uinstr_rdy
  };

  //--------------------------------------------
  // Collect mvimm
  //--------------------------------------------
  // fif3_imm_uinstr
  assign fif3_imm_uinstr = txu_au_pkg::imm_uinstr_s'(fif2_dn_rcp_instr[IMM_UINSTR_S_WIDTH-1:0]);

  // s2p for dn_mvimm_out
  p2p_if_s2p2_comb_init #(
      .DATA_WIDTH(IMM_UINSTR_S_WIDTH)
  ) u_p2p_if_s2p2 (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_init),

      // p2p target interface
      .up_data(fif3_imm_uinstr),
      .up_vld (fif3_imm_uinstr_val),
      .up_rdy (fif3_imm_uinstr_rdy),

      // p2p initiator interface
      .dn_data(fif3_dn_imm_uinstr_vec),
      .dn_vld (fif3_dn_imm_uinstr_vec_val),
      .dn_rdy (fif3_dn_imm_uinstr_vec_rdy)
  );  // p2p_if_s2p2_comb_init u_p2p_if_s2p2

  //--------------------------------------------
  // Collect mvrimm
  //--------------------------------------------
  // fif3_rimm_uinstr
  assign fif3_rimm_uinstr = txu_au_pkg::rimm_uinstr_s'(fif2_dn_rcp_instr[RIMM_UINSTR_S_WIDTH-1:0]);

  // fif3_rimm_uinstr_cmb_val
  assign fif3_rimm_uinstr_cmb_val = fif3_rimm_uinstr_val & rimm_data_val;

  // fif3_rimm_uinstr_rdy
  assign fif3_rimm_uinstr_rdy = fif3_rimm_uinstr_cmb_rdy & fif3_rimm_uinstr_cmb_val;

  // fif3_dn_rimm_uinstr_vec
  p2p_if_pipe_init #(
      .DATA_WIDTH(RIP_DATA_WIDTH + RIMM_UINSTR_S_WIDTH),
      .BYPASS    (BYPASS)                                 // NOTE
  ) u_p2p_if_pipe_rimm (
      // clock and reset
      .clk,
      .rst_n,

      // init
      .init(uinstr_queue_init),

      // p2p target interface
      .up_data({rimm_data, fif3_rimm_uinstr}),
      .up_vld (fif3_rimm_uinstr_cmb_val),
      .up_rdy (fif3_rimm_uinstr_cmb_rdy),

      // p2p initiator interface
      .dn_data(fif3_dn_rimm_uinstr_vec),
      .dn_vld (fif3_dn_rimm_uinstr_vec_val),
      .dn_rdy (fif3_dn_rimm_uinstr_vec_rdy)
  );  // p2p_if_pipe_init u_p2p_if_pipe_rimm

  //--------------------------------------------
  // Ctl
  //--------------------------------------------
  // fif3_dn_uau_ctl
  always_ff @(posedge clk, negedge rst_n) begin : ff_fif3_dn_uau_ctl
    if (!rst_n) begin
      fif3_dn_uau_ctl <= '0;
    end else if (fif3_ctl_uinstr_val && fif3_ctl_uinstr_rdy && (fif2_dn_rcp_instr.vr_enc.ctl.enc1 == UAU_CTL)) begin
      fif3_dn_uau_ctl <= fif2_dn_rcp_instr.vr_enc.ctl.params[UAU_CTL_S_WIDTH-1:0];
    end
  end : ff_fif3_dn_uau_ctl

  // fif3_dn_upwr_req_ctl
  always_ff @(posedge clk, negedge rst_n) begin : ff_fif3_dn_upwr_req_ctl
    if (!rst_n) begin
      fif3_dn_upwr_req_ctl <= UPWR_REQ_CTL_S_WIDTH'(7'h40);
    end else if (fif3_ctl_uinstr_val && fif3_ctl_uinstr_rdy_loc && (fif2_dn_rcp_instr.vr_enc.ctl.enc1 == UPWR_REQ_CTL)) begin
      fif3_dn_upwr_req_ctl <= fif2_dn_rcp_instr.vr_enc.ctl.params[UPWR_REQ_CTL_S_WIDTH-1:0];
    end
  end : ff_fif3_dn_upwr_req_ctl

  //--------------------------------------------
  // miu scbd lock
  //--------------------------------------------
  // fif2_miu_if0_up_en
  assign fif2_miu_if0_up_en = miu_uinstr_val;

  // fif2_miu_scbd_lock_wr_[idx|en]_loc
  assign fif2_miu_scbd_lock_wr_idx_loc[PORT0] = scbd_vd1_idx & {VRF_IDX_WIDTH{fif2_miu_if0_up_en}};
  assign fif2_miu_scbd_lock_wr_idx_loc[PORT1] = '0;

  assign fif2_miu_scbd_lock_wr_en_loc[PORT0] = scbd_vd1_idx_en & fif2_miu_if0_up_en;
  assign fif2_miu_scbd_lock_wr_en_loc[PORT1] = 1'b0;

  // fif2_miu_scbd_fe_lock_wr_[idx|en]
  assign fif2_miu_scbd_fe_lock_wr_idx[PORT0] =
    miu_scbd_fe_lock_wr_idx[PORT0] & {VRF_IDX_WIDTH{miu_scbd_fe_lock_wr_en[PORT0]}};
  assign fif2_miu_scbd_fe_lock_wr_idx[PORT1] = '0;

  assign fif2_miu_scbd_fe_lock_wr_en[PORT0] = miu_scbd_fe_lock_wr_en[PORT0];
  assign fif2_miu_scbd_fe_lock_wr_en[PORT1] = 1'b0;

  // fif2_miu_scbd_lock_wr_[idx|en]
  assign fif2_miu_scbd_lock_wr_idx[PORT0] =
    fif2_miu_scbd_lock_wr_idx_loc[PORT0] | fif2_miu_scbd_fe_lock_wr_idx[PORT0];
  assign fif2_miu_scbd_lock_wr_idx[PORT1] = '0;

  assign fif2_miu_scbd_lock_wr_en[PORT0] =
    fif2_miu_scbd_lock_wr_en_loc[PORT0] | fif2_miu_scbd_fe_lock_wr_en[PORT0];
  assign fif2_miu_scbd_lock_wr_en[PORT1] = 1'b0;

  //--------------------------------------------
  // x0 scbd lock
  //--------------------------------------------
  // fif2_x0_if0_up_en
  assign fif2_x0_if0_up_en = x0_uinstr_val;

  // fif2_x0_scbd_lock_wr_[idx|en]_loc
  assign fif2_x0_scbd_lock_wr_idx_loc[PORT0] = scbd_vd1_idx & {VRF_IDX_WIDTH{fif2_x0_if0_up_en}};
  assign fif2_x0_scbd_lock_wr_idx_loc[PORT1] = scbd_vd2_idx & {VRF_IDX_WIDTH{fif2_x0_if0_up_en}};

  assign fif2_x0_scbd_lock_wr_en_loc[PORT0] = scbd_vd1_idx_en & fif2_x0_if0_up_en;
  assign fif2_x0_scbd_lock_wr_en_loc[PORT1] = scbd_vd2_idx_en & fif2_x0_if0_up_en;

  // fif2_x0_scbd_fe_lock_wr_[idx|en]
  assign fif2_x0_scbd_fe_lock_wr_idx[PORT0] =
    x0_scbd_fe_lock_wr_idx[PORT0] & {VRF_IDX_WIDTH{x0_scbd_fe_lock_wr_en[PORT0]}};
  assign fif2_x0_scbd_fe_lock_wr_idx[PORT1] =
    x0_scbd_fe_lock_wr_idx[PORT1] & {VRF_IDX_WIDTH{x0_scbd_fe_lock_wr_en[PORT1]}};

  assign fif2_x0_scbd_fe_lock_wr_en[PORT0] = x0_scbd_fe_lock_wr_en[PORT0];
  assign fif2_x0_scbd_fe_lock_wr_en[PORT1] = x0_scbd_fe_lock_wr_en[PORT1];

  // fif2_x0_scbd_lock_wr_[idx|en]
  assign fif2_x0_scbd_lock_wr_idx[PORT0] =
    fif2_x0_scbd_lock_wr_idx_loc[PORT0] | fif2_x0_scbd_fe_lock_wr_idx[PORT0];
  assign fif2_x0_scbd_lock_wr_idx[PORT1] =
    fif2_x0_scbd_lock_wr_idx_loc[PORT1] | fif2_x0_scbd_fe_lock_wr_idx[PORT1];

  assign fif2_x0_scbd_lock_wr_en[PORT0] =
    fif2_x0_scbd_lock_wr_en_loc[PORT0] | fif2_x0_scbd_fe_lock_wr_en[PORT0];
  assign fif2_x0_scbd_lock_wr_en[PORT1] =
    fif2_x0_scbd_lock_wr_en_loc[PORT1] | fif2_x0_scbd_fe_lock_wr_en[PORT1];

  //--------------------------------------------
  // x1 scbd lock
  //--------------------------------------------
  // fif2_x1_if0_up_en
  assign fif2_x1_if0_up_en = x1_uinstr_val;

  // fif2_x1_scbd_lock_wr_[idx|en]_loc
  assign fif2_x1_scbd_lock_wr_idx_loc[PORT0] = scbd_vd1_idx & {VRF_IDX_WIDTH{fif2_x1_if0_up_en}};
  assign fif2_x1_scbd_lock_wr_idx_loc[PORT1] = '0;

  assign fif2_x1_scbd_lock_wr_en_loc[PORT0] = scbd_vd1_idx_en & fif2_x1_if0_up_en;
  assign fif2_x1_scbd_lock_wr_en_loc[PORT1] = 1'b0;

  // fif2_x1_scbd_fe_lock_wr_[idx|en]
  assign fif2_x1_scbd_fe_lock_wr_idx[PORT0] =
    x1_scbd_fe_lock_wr_idx[PORT0] & {VRF_IDX_WIDTH{x1_scbd_fe_lock_wr_en[PORT0]}};
  assign fif2_x1_scbd_fe_lock_wr_idx[PORT1] = '0;

  assign fif2_x1_scbd_fe_lock_wr_en[PORT0] = x1_scbd_fe_lock_wr_en[PORT0];
  assign fif2_x1_scbd_fe_lock_wr_en[PORT1] = 1'b0;

  // fif2_x1_scbd_lock_wr_[idx|en]
  assign fif2_x1_scbd_lock_wr_idx[PORT0] = '0;
  assign fif2_x1_scbd_lock_wr_idx[PORT1] =
    fif2_x1_scbd_lock_wr_idx_loc[PORT0] | fif2_x1_scbd_fe_lock_wr_idx[PORT0];

  assign fif2_x1_scbd_lock_wr_en[PORT0] = 1'b0;
  assign fif2_x1_scbd_lock_wr_en[PORT1] =
    fif2_x1_scbd_lock_wr_en_loc[PORT0] | fif2_x1_scbd_fe_lock_wr_en[PORT0];

  //--------------------------------------------
  // x2 scbd lock
  //--------------------------------------------
  // fif2_x2_if0_up_en
  assign fif2_x2_if0_up_en = x2_uinstr_val;

  // fif2_x2_scbd_lock_wr_[idx|en]_loc
  assign fif2_x2_scbd_lock_wr_idx_loc[PORT0] = scbd_vd1_idx & {VRF_IDX_WIDTH{fif2_x2_if0_up_en}};
  assign fif2_x2_scbd_lock_wr_idx_loc[PORT1] = scbd_vd2_idx & {VRF_IDX_WIDTH{fif2_x2_if0_up_en}};

  assign fif2_x2_scbd_lock_wr_en_loc[PORT0] = scbd_vd1_idx_en & fif2_x2_if0_up_en;
  assign fif2_x2_scbd_lock_wr_en_loc[PORT1] = scbd_vd2_idx_en & fif2_x2_if0_up_en;

  // fif2_x2_scbd_fe_lock_wr_[idx|en]
  assign fif2_x2_scbd_fe_lock_wr_idx[PORT0] =
    x2_scbd_fe_lock_wr_idx[PORT0] & {VRF_IDX_WIDTH{x2_scbd_fe_lock_wr_en[PORT0]}};
  assign fif2_x2_scbd_fe_lock_wr_idx[PORT1] =
    x2_scbd_fe_lock_wr_idx[PORT1] & {VRF_IDX_WIDTH{x2_scbd_fe_lock_wr_en[PORT1]}};

  assign fif2_x2_scbd_fe_lock_wr_en[PORT0] = x2_scbd_fe_lock_wr_en[0];
  assign fif2_x2_scbd_fe_lock_wr_en[PORT1] = x2_scbd_fe_lock_wr_en[1];

  // fif2_x2_scbd_lock_wr_[idx|en]
  assign fif2_x2_scbd_lock_wr_idx[PORT1] =
    fif2_x2_scbd_lock_wr_idx_loc[PORT1] | fif2_x2_scbd_fe_lock_wr_idx[PORT1];
  assign fif2_x2_scbd_lock_wr_idx[PORT0] =
    fif2_x2_scbd_lock_wr_idx_loc[PORT0] | fif2_x2_scbd_fe_lock_wr_idx[PORT0];

  assign fif2_x2_scbd_lock_wr_en[PORT1] =
    fif2_x2_scbd_lock_wr_en_loc[PORT1] | fif2_x2_scbd_fe_lock_wr_en[PORT1];
  assign fif2_x2_scbd_lock_wr_en[PORT0] =
    fif2_x2_scbd_lock_wr_en_loc[PORT0] | fif2_x2_scbd_fe_lock_wr_en[PORT0];

  //--------------------------------------------
  // Outputs
  //--------------------------------------------
  //-- cp_instr target interface ---------------
  // assign cp_idle = 1'b0;
  assign cp_dirty = 1'b0;

  //-- scbd read target interface --------------
  assign scbd = fif3_scbd;

  //-- rimm data target interface --------------
  assign rimm_data_rdy = fif3_rimm_uinstr_rdy;

  //-- scbd read config initiator interface ----
  assign scbd_vs1_idx = fif2_dn_scbd_vs1_idx;
  assign scbd_vs1_idx_en = fif2_dn_scbd_vs1_idx_en;
  assign scbd_vs2_idx = fif2_dn_scbd_vs2_idx;
  assign scbd_vs2_idx_en = fif2_dn_scbd_vs2_idx_en;

  //-- scbd write config initiator interface ---
  assign scbd_vd1_idx = fif2_dn_scbd_vd1_idx;
  assign scbd_vd1_idx_en = fif2_dn_scbd_vd1_idx_en;
  assign scbd_vd2_idx = fif2_dn_scbd_vd2_idx;
  assign scbd_vd2_idx_en = fif2_dn_scbd_vd2_idx_en;

  //-- rip initiator0 interface ----------------
  assign rip_mst0_uicc_val = fif3_dn_imm_uinstr_vec_val;
  assign fif3_dn_imm_uinstr_vec_rdy = rip_mst0_uicc_rdy;
  assign rip_mst0_uicc_addr = {fif3_dn_imm_uinstr_vec.high.rcp, fif3_dn_imm_uinstr_vec.low.rcp};
  assign rip_mst0_uicc_wdata = {fif3_dn_imm_uinstr_vec.high.imm, fif3_dn_imm_uinstr_vec.low.imm};
  assign rip_mst0_uicc_wen = 1'b1;
  assign rip_mst0_uicc_ben = 4'hF;

  //-- rip initiator1 interface ----------------
  assign rip_mst1_uicc_val = fif3_dn_rimm_uinstr_vec_val;
  assign fif3_dn_rimm_uinstr_vec_rdy = rip_mst1_uicc_rdy;
  assign rip_mst1_uicc_addr = {fif3_dn_rimm_uinstr_vec.low.bid, fif3_dn_rimm_uinstr_vec.low.rid};
  assign rip_mst1_uicc_wdata = fif3_dn_rimm_uinstr_vec.high;
  assign rip_mst1_uicc_wen = 1'b1;
  assign rip_mst1_uicc_ben = 4'hF;

  //-- ctl initiator interface -----------------
  assign uau_ctl = fif3_dn_uau_ctl;
  assign upwr_req_ctl = fif3_dn_upwr_req_ctl;
  assign miu_overlap_en = fif2_dn_miu_overlap_en;
  assign miu_overlap_idx_sel = fif2_dn_miu_overlap_idx_sel;

  //-- [miu,x0,x1,x2] uinstr initiator if ------
  assign miu_uinstr = fif2_dn_rcp_instr[UINSTR_WIDTH-1:0];
  assign miu_st_uinstr = fif2_dn_rcp_instr[UINSTR_WIDTH-1:0];
  assign x0_uinstr = fif2_dn_rcp_instr[UINSTR_WIDTH-1:0];
  assign x1_uinstr = fif2_dn_rcp_instr[UINSTR_WIDTH-1:0];
  assign x2_uinstr = fif2_dn_rcp_instr[UINSTR_WIDTH-1:0];

  assign miu_uinstr_val = fif3_miu_uinstr_val & fif3_miu_uinstr_rdy;
  assign miu_st_uinstr_val = fif3_miu_st_uinstr_val & fif3_miu_st_uinstr_rdy;
  assign x0_uinstr_val = fif3_x0_uinstr_val & fif3_x0_uinstr_rdy;
  assign x1_uinstr_val = fif3_x1_uinstr_val & fif3_x1_uinstr_rdy;
  assign x2_uinstr_val = fif3_x2_uinstr_val & fif3_x2_uinstr_rdy;

  //-- au config initiator interface -----------
  always_ff @(posedge clk, negedge rst_n) begin : ff_uinstr_cnt
    if (!rst_n) begin
      uinstr_cnt    <= '0;
      uinstr_cnt_en <= 1'b0;
    end else if (uinstr_queue_init) begin
      uinstr_cnt    <= '0;
      uinstr_cnt_en <= 1'b0;
    end else if (profile_start_trig) begin
      uinstr_cnt    <= '0;
      uinstr_cnt_en <= 1'b1;
    end else if (profile_end_trig) begin
      uinstr_cnt_en <= 1'b0;
    end else if (uinstr_cnt_en) begin
      uinstr_cnt <= UINSTR_CNT_WIDTH'(uinstr_cnt + UINSTR_CNT_WIDTH'(1'b1));
    end
  end : ff_uinstr_cnt

  //-- uinstr queue status initiator interface -
  assign uinstr_queue_is_empty = fif0_dn_uinstr_queue_is_empty;
  assign uinstr_queue_is_full  = fif0_dn_uinstr_queue_is_full;

  //-- monitor initiator interface -------------
  // mon_uinstr_queue_cnt
  always_ff @(posedge clk, negedge rst_n) begin : always_mon_sigs
    if (!rst_n) begin
      mon_uinstr_queue_cnt <= '0;  // FIXME
    end else if (fif2_dn_rcp_instr_val && fif2_dn_rcp_instr_rdy) begin
      mon_uinstr_queue_cnt <= uinstr_cnt;
    end
  end : always_mon_sigs

  // mon_rcp_instr
  assign mon_rcp_instr = RIP_DATA_WIDTH'({fif2_dn_rcp_instr, 7'h2B});

  // mon_scbd_l, mon_scbd_h
  always_ff @(posedge clk, negedge rst_n) begin : ff_mon_scbd
    if (!rst_n) begin
      mon_scbd0 <= '0;
      mon_scbd1 <= '0;
      mon_scbd2 <= '0;
      mon_scbd3 <= '0;
    end else if (fif2_dn_rcp_instr_val && fif2_dn_rcp_instr_rdy) begin
      {mon_scbd3, mon_scbd2, mon_scbd1, mon_scbd0} <= scbd;
    end
  end : ff_mon_scbd

  //--------------------------------------------
  // DEBUG
  //--------------------------------------------
  // synopsys translate_off

  always_comb begin : comb_debug
    import txu_au_pkg::UINSTR_SU_ENC_MASK;
    import txu_au_pkg::UINSTR_MIU_ENC_MASK;
    import txu_au_pkg::UINSTR_UMEM_ENC_MASK;
    import txu_au_pkg::UINSTR_EU_ENC_MASK;

    import txu_au_pkg::INVALID_SU_ENC_BASE;
    import txu_au_pkg::INVALID_MIU_ENC_BASE;
    import txu_au_pkg::INVALID_UMEM_ENC_BASE;
    import txu_au_pkg::INVALID_EU_ENC_BASE;

    logic                                                     dbg_miu_phs;
    logic                                                     dbg_x0_phs;
    logic                                                     dbg_x1_phs;
    logic                                                     dbg_x2_phs;

    logic                                                     dbg_miu_uinstr_en;
    logic                                                     dbg_x0_uinstr_en;
    logic                                                     dbg_x1_uinstr_en;
    logic                                                     dbg_x2_uinstr_en;

    logic                                                     dbg_rip_mst0_uicc_en;
    logic                                                     dbg_rip_mst1_uicc_en;

    logic                              [CP_INSTR_WIDTH - 1:0] dbg_dn_rcp_instr;
    logic                                                     dbg_dn_rcp_instr_en;
    txu_au_pkg::uinstr_sel_e                                  dbg_uinstr_sel;

    txu_au_pkg::uinstr_miu_enc_base_e                         dbg_cp_miu_instr;
    txu_au_pkg::uinstr_umem_enc_base_e                        dbg_cp_umem_instr;
    txu_au_pkg::uinstr_enc_base_e                             dbg_cp_eu_instr;

    // miu
    dbg_miu_uinstr_en = miu_uinstr_val & miu_uinstr_rdy;
    dbg_miu_phs = is_uop_phs1;

    // x0
    dbg_x0_uinstr_en = x0_uinstr_val & x0_uinstr_rdy;
    dbg_x0_phs = is_uop_phs2;

    // x1
    dbg_x1_uinstr_en = x1_uinstr_val & miu_uinstr_rdy;
    dbg_x1_phs = is_uop_phs3;

    // x2
    dbg_x2_uinstr_en = x2_uinstr_val & x0_uinstr_rdy;
    dbg_x2_phs = is_uop_phs0;

    // rip0
    dbg_rip_mst0_uicc_en = rip_mst0_uicc_val & rip_mst0_uicc_rdy;

    // rip1
    dbg_rip_mst1_uicc_en = rip_mst1_uicc_val & rip_mst1_uicc_rdy;

    // dbg_dn_rcp_instr
    // dbg_dn_rcp_instr = {fif2_dn_rcp_instr, 7'h2B};
    dbg_dn_rcp_instr = cp_instr;

    // dbg_dn_rcp_instr_en
    dbg_dn_rcp_instr_en = fif2_dn_rcp_instr_val & fif2_dn_rcp_instr_rdy;

    // dbg_uinstr_sel
    dbg_uinstr_sel = txu_au_pkg::uinstr_sel_e'(fif3_uinstr_sel);

    // dbg_cp_miu_instr
    dbg_cp_miu_instr = txu_au_pkg::uinstr_miu_enc_base_e'(dbg_dn_rcp_instr & UINSTR_MIU_ENC_MASK);
    if (dbg_cp_miu_instr.name() == "") begin
      dbg_cp_miu_instr = INVALID_MIU_ENC_BASE;
    end

    // dbg_cp_umem_instr
    dbg_cp_umem_instr = txu_au_pkg::uinstr_umem_enc_base_e'(dbg_dn_rcp_instr & UINSTR_UMEM_ENC_MASK);
    if (dbg_cp_umem_instr.name() == "") begin
      dbg_cp_umem_instr = INVALID_UMEM_ENC_BASE;
    end

    // dbg_cp_eu_instr
    dbg_cp_eu_instr = txu_au_pkg::uinstr_enc_base_e'(dbg_dn_rcp_instr & UINSTR_EU_ENC_MASK);
    if (dbg_cp_eu_instr.name() == "") begin
      dbg_cp_eu_instr = INVALID_EU_ENC_BASE;
    end
  end : comb_debug

  // synopsys translate_on

endmodule : txu_uicc
