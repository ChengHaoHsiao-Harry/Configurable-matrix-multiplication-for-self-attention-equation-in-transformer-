`ifndef _common_vh
`define _common_vh

`define SRAM_ADDR_WIDTH       		16
`define SRAM_ADDR_LSB          		0
`define SRAM_ADDR_MSB          		`SRAM_ADDR_WIDTH-1
`define SRAM_ADDR_RANGE        		`SRAM_ADDR_MSB : `SRAM_ADDR_LSB

`define SRAM_DATA_WIDTH           	32
`define SRAM_DATA_LSB             	0
`define SRAM_DATA_MSB             	`SRAM_DATA_WIDTH-1
`define SRAM_DATA_RANGE           	`SRAM_DATA_MSB : `SRAM_DATA_LSB

`endif
