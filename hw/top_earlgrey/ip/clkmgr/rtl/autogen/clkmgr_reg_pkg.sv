// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Package auto-generated by `reggen` containing data structure

package clkmgr_reg_pkg;

  // Param list
  parameter int NumGroups = 7;
  parameter int NumAlerts = 1;

  // Address widths within the block
  parameter int BlockAw = 5;

  ////////////////////////////
  // Typedefs for registers //
  ////////////////////////////

  typedef struct packed {
    logic        q;
    logic        qe;
  } clkmgr_reg2hw_alert_test_reg_t;

  typedef struct packed {
    logic [3:0]  q;
  } clkmgr_reg2hw_extclk_sel_reg_t;

  typedef struct packed {
    logic        q;
  } clkmgr_reg2hw_jitter_enable_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } clk_io_div4_peri_en;
    struct packed {
      logic        q;
    } clk_io_div2_peri_en;
    struct packed {
      logic        q;
    } clk_io_peri_en;
    struct packed {
      logic        q;
    } clk_usb_peri_en;
  } clkmgr_reg2hw_clk_enables_reg_t;

  typedef struct packed {
    struct packed {
      logic        q;
    } aes_hint;
    struct packed {
      logic        q;
    } hmac_hint;
    struct packed {
      logic        q;
    } kmac_hint;
    struct packed {
      logic        q;
    } otbn_hint;
  } clkmgr_reg2hw_clk_hints_reg_t;

  typedef struct packed {
    struct packed {
      logic        d;
      logic        de;
    } aes_val;
    struct packed {
      logic        d;
      logic        de;
    } hmac_val;
    struct packed {
      logic        d;
      logic        de;
    } kmac_val;
    struct packed {
      logic        d;
      logic        de;
    } otbn_val;
  } clkmgr_hw2reg_clk_hints_status_reg_t;

  // Register -> HW type
  typedef struct packed {
    clkmgr_reg2hw_alert_test_reg_t alert_test; // [14:13]
    clkmgr_reg2hw_extclk_sel_reg_t extclk_sel; // [12:9]
    clkmgr_reg2hw_jitter_enable_reg_t jitter_enable; // [8:8]
    clkmgr_reg2hw_clk_enables_reg_t clk_enables; // [7:4]
    clkmgr_reg2hw_clk_hints_reg_t clk_hints; // [3:0]
  } clkmgr_reg2hw_t;

  // HW -> register type
  typedef struct packed {
    clkmgr_hw2reg_clk_hints_status_reg_t clk_hints_status; // [7:0]
  } clkmgr_hw2reg_t;

  // Register offsets
  parameter logic [BlockAw-1:0] CLKMGR_ALERT_TEST_OFFSET = 5'h 0;
  parameter logic [BlockAw-1:0] CLKMGR_EXTCLK_SEL_REGWEN_OFFSET = 5'h 4;
  parameter logic [BlockAw-1:0] CLKMGR_EXTCLK_SEL_OFFSET = 5'h 8;
  parameter logic [BlockAw-1:0] CLKMGR_JITTER_ENABLE_OFFSET = 5'h c;
  parameter logic [BlockAw-1:0] CLKMGR_CLK_ENABLES_OFFSET = 5'h 10;
  parameter logic [BlockAw-1:0] CLKMGR_CLK_HINTS_OFFSET = 5'h 14;
  parameter logic [BlockAw-1:0] CLKMGR_CLK_HINTS_STATUS_OFFSET = 5'h 18;

  // Reset values for hwext registers and their fields
  parameter logic [0:0] CLKMGR_ALERT_TEST_RESVAL = 1'h 0;
  parameter logic [0:0] CLKMGR_ALERT_TEST_FATAL_FAULT_RESVAL = 1'h 0;

  // Register index
  typedef enum int {
    CLKMGR_ALERT_TEST,
    CLKMGR_EXTCLK_SEL_REGWEN,
    CLKMGR_EXTCLK_SEL,
    CLKMGR_JITTER_ENABLE,
    CLKMGR_CLK_ENABLES,
    CLKMGR_CLK_HINTS,
    CLKMGR_CLK_HINTS_STATUS
  } clkmgr_id_e;

  // Register width information to check illegal writes
  parameter logic [3:0] CLKMGR_PERMIT [7] = '{
    4'b 0001, // index[0] CLKMGR_ALERT_TEST
    4'b 0001, // index[1] CLKMGR_EXTCLK_SEL_REGWEN
    4'b 0001, // index[2] CLKMGR_EXTCLK_SEL
    4'b 0001, // index[3] CLKMGR_JITTER_ENABLE
    4'b 0001, // index[4] CLKMGR_CLK_ENABLES
    4'b 0001, // index[5] CLKMGR_CLK_HINTS
    4'b 0001  // index[6] CLKMGR_CLK_HINTS_STATUS
  };

endpackage

