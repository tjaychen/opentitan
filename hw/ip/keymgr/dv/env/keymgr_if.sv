// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// interface for input data from LC, OTP and flash
interface keymgr_if(input clk, input rst_n);

  import keymgr_pkg::*;

  lc_ctrl_pkg::lc_keymgr_div_t keymgr_div;
  otp_data_t otp;
  otp_ctrl_pkg::otp_keymgr_key_t otp_key;
  flash_ctrl_pkg::keymgr_flash_t flash;
  edn_pkg::edn_rsp_t edn_rsp;

  hw_key_req_t kmac_key;
  hw_key_req_t hmac_key;
  hw_key_req_t aes_key;

  task automatic init();
    keymgr_div = 64'h5CFBD765CE33F34E;
    otp     = OTP_DATA_DEFAULT;
    otp_key = otp_ctrl_pkg::OTP_KEYMGR_KEY_DEFAULT;
    flash   = flash_ctrl_pkg::KEYMGR_FLASH_DEFAULT;
    edn_rsp = '{
      edn_ack: 1'b1,
      edn_fips: 1'b0,
      edn_bus: 32'hFBA6E5DA
    };

  endtask
endinterface
