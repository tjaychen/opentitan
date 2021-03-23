# Copyright lowRISC contributors.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

set workroot [file dirname [info script]]

send_msg "Designcheck 1-1" INFO "Checking design"

# Ensure the design meets timing
set slack_ns [get_property SLACK [get_timing_paths -delay_type min_max]]
send_msg "Designcheck 1-2" INFO "Slack is ${slack_ns} ns."

if [expr {$slack_ns < 0}] {
  send_msg "Designcheck 1-3" ERROR "Timing failed. Slack is ${slack_ns} ns."
}

# Enable bitstream identification via USR_ACCESS register.
set_property BITSTREAM.CONFIG.USR_ACCESS TIMESTAMP [current_design]

set filename "${workroot}/rom.mmi"
set fileout [open $filename "w"]
set brams [split [get_cells -hierarchical -filter { PRIMITIVE_TYPE =~ BMEM.bram.* && NAME =~ *u_rom_rom*}] " "]

# Should merge the two loops later
# calculate the overall address space
set space 0
foreach inst [lsort -dictionary $brams] {
  set slice_begin [get_property bram_slice_begin [get_cells $inst]]
  set slice_end [get_property bram_slice_end [get_cells $inst]]
  set addr_begin [get_property bram_addr_begin [get_cells $inst]]
  set addr_end [get_property bram_addr_end [get_cells $inst]]
  # calculate total number of bits
  set space [expr {$space + ($addr_end - $addr_begin + 1) * ($slice_end - $slice_begin + 1)}]
}

set space [expr {($space / 8) - 1}]
puts $fileout "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
puts $fileout "<MemInfo Version=\"1\" Minor=\"0\">"
puts $fileout "  <Processor Endianness=\"Little\" InstPath=\"dummy\">"
puts $fileout "  <AddressSpace Name=\"brom\" Begin=\"0\" End=\"$space\">"
puts $fileout "      <BusBlock>"

set part [get_property PART [current_design]]
foreach inst [lsort -dictionary $brams] {
  set loc [get_property LOC [get_cells $inst]]
  set loc [string trimleft $loc RAMB36_]
  set slice_begin [get_property bram_slice_begin [get_cells $inst]]
  set slice_end [get_property bram_slice_end [get_cells $inst]]
  set addr_begin [get_property bram_addr_begin [get_cells $inst]]
  set addr_end [get_property bram_addr_end [get_cells $inst]]
  puts $fileout "        <BitLane MemType=\"RAMB32\" Placement=\"$loc\">"
  puts $fileout "          <DataWidth MSB=\"$slice_end\" LSB=\"$slice_begin\"/>"
  puts $fileout "          <AddressRange Begin=\"$addr_begin\" End=\"$addr_end\"/>"
  puts $fileout "          <Parity ON=\"false\" NumBits=\"0\"/>"
  puts $fileout "        </BitLane>"
}
puts $fileout "      </BusBlock>"
puts $fileout "    </AddressSpace>"
puts $fileout "  </Processor>"
puts $fileout "<Config>"
puts $fileout "  <Option Name=\"Part\" Val=\"$part\"/>"
puts $fileout "</Config>"
puts $fileout "</MemInfo>"
close $fileout
