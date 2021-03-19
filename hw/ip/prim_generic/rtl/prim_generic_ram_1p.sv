// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Synchronous single-port SRAM model

`include "prim_assert.sv"

module prim_generic_ram_1p import prim_ram_1p_pkg::*; #(
  parameter  int Width           = 32, // bit
  parameter  int Depth           = 128,
  parameter  int DataBitsPerMask = 1, // Number of data bits per bit of write mask
  parameter      MemInitFile     = "", // VMEM file to initialize the memory with

  localparam int Aw              = $clog2(Depth)  // derived parameter
) (
  input  logic             clk_i,

  input  logic             req_i,
  input  logic             write_i,
  input  logic [Aw-1:0]    addr_i,
  input  logic [Width-1:0] wdata_i,
  input  logic [Width-1:0] wmask_i,
  output logic [Width-1:0] rdata_o, // Read data. Data is returned one cycle after req_i is high.
  input ram_1p_cfg_t       cfg_i
);

  logic unused_cfg;
  assign unused_cfg = ^cfg_i;

//  // The width of the word may not always align with the bitmask.
//  // Such situations can arise if the upper level incorporates a protocol such as
//  // memory scrambling, which does not work if DataBitsPerMask is 1.
//  // When this happens, allow the upper layers to use DataBitsPerMask > 1,
//  // but convert back to a bit mask of 1 for memory updating purposes.
//  localparam bit AlignedMask = (Width % DataBitsPerMask) == 0;
//
//  // Width of internal write mask. Note wmask_i input into the module is always assumed
//  // to be the full bit mask
//  localparam int LocalDataBitsPerMask = AlignedMask ? DataBitsPerMask : 1;
//  localparam int MaskWidth = Width / LocalDataBitsPerMask;
//
//  logic [Width-1:0]     mem [Depth];
//  logic [MaskWidth-1:0] wmask;
//
//  for (genvar k = 0; k < MaskWidth; k++) begin : gen_wmask
//    assign wmask[k] = &wmask_i[k*LocalDataBitsPerMask +: LocalDataBitsPerMask];
//
//    // Ensure that all mask bits within a group have the same value for a write
//    `ASSERT(MaskCheck_A, req_i && write_i |->
//        wmask_i[k*LocalDataBitsPerMask +: LocalDataBitsPerMask] inside
//        {{LocalDataBitsPerMask{1'b1}}, '0}, clk_i, '0)
//  end
//
//  // using always instead of always_ff to avoid 'ICPD  - illegal combination of drivers' error
//  // thrown when using $readmemh system task to backdoor load an image
//  always @(posedge clk_i) begin
//    if (req_i) begin
//      if (write_i) begin
//        for (int i=0; i < MaskWidth; i = i + 1) begin
//          if (wmask[i]) begin
//            mem[addr_i][i*LocalDataBitsPerMask +: LocalDataBitsPerMask] <=
//              wdata_i[i*LocalDataBitsPerMask +: LocalDataBitsPerMask];
//          end
//        end
//      end else begin
//        rdata_o <= mem[addr_i];
//      end
//    end
//  end


  localparam bit Unaligned = (Width % DataBitsPerMask) > 0;
  // The '1' given here is an arbitrary value to silence lint errors.
  localparam int Remainder = Unaligned ? Width % DataBitsPerMask : DataBitsPerMask;
  localparam int MaskWidth = prim_util_pkg::div_round_up(Width, DataBitsPerMask);
  //localparam int AlignedWidth = MaskWidth * DataBitsPerMask;

  logic [Width-1:0]     mem [Depth];
  logic [MaskWidth-1:0] wmask;
  //logic [MaskWidth-1:0] wmask_unaligned;
  //
  //logic [AlignedWidth-1:0] wdata_aligned;
  //assign wdata_aligned = AlignedWidth'(wdata_i);

  for (genvar k = 0; k < MaskWidth; k++) begin : gen_wmask
    if (k == MaskWidth-1 && Unaligned) begin : gen_last_value
      assign wmask[k] = &wmask_i[k*DataBitsPerMask +: Remainder];
      //assign wmask_unaligned[k] = wmask[k];
    end else begin : gen_normal_value
      assign wmask[k] = &wmask_i[k*DataBitsPerMask +: DataBitsPerMask];
      //assign wmask_unaligned[k] = 1'b0;
    end
  end

  logic [Width-1:0] mem_in;
  for (genvar i = 0; i < MaskWidth; i++ ) begin : gen_mem_in
    if (i == MaskWidth - 1) begin : gen_stuff
      assign mem_in[i*DataBitsPerMask +: Remainder] =
        wmask[i] ? wdata_i[i*DataBitsPerMask +: Remainder] :
                   mem[addr_i][i*DataBitsPerMask +: Remainder];
    end else begin : gen_non_stuff
      assign mem_in[i*DataBitsPerMask +: DataBitsPerMask] =
        wmask[i] ? wdata_i[i*DataBitsPerMask +: DataBitsPerMask] :
                   mem[addr_i][i*DataBitsPerMask +: DataBitsPerMask];
    end
  end

  // using always instead of always_ff to avoid 'ICPD  - illegal combination of drivers' error
  // thrown when using $readmemh system task to backdoor load an image
  always @(posedge clk_i) begin
    if (req_i) begin
      if (write_i) begin
        mem[addr_i] <= mem_in;
      end else begin
        rdata_o <= mem[addr_i];
      end
    end
  end



  //always @(posedge clk_i) begin
  //  if (req_i) begin
  //    if (write_i) begin
  //      for (int i=0; i < MaskWidth; i = i + 1) begin
  //        if (i == (MaskWidth - 1) && wmask[i]) begin
  //          mem[addr_i][i*DataBitsPerMask +: Remainder] <=
  //            wdata_i[i*DataBitsPerMask +: Remainder];
  //        end else if (wmask[i]) begin
  //          mem[addr_i][i*DataBitsPerMask +: DataBitsPerMask] <=
  //            wdata_i[i*DataBitsPerMask +: DataBitsPerMask];
  //        end
  //
  //
  //        //if (wmask_unaligned[i] & wmask[i]) begin
  //        //  mem[addr_i][i*DataBitsPerMask +: Remainder] <=
  //        //    wdata_aligned[i*DataBitsPerMask +: Remainder];
  //        //end else if (wmask[i]) begin
  //        //  mem[addr_i][i*DataBitsPerMask +: DataBitsPerMask] <=
  //        //    wdata_aligned[i*DataBitsPerMask +: DataBitsPerMask];
  //        //end
  //      end
  //    end else begin
  //      rdata_o <= mem[addr_i];
  //    end
  //  end
  //end


  `include "prim_util_memload.svh"
endmodule
