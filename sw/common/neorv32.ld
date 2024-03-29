/* ################################################################################################# */
/* # << NEORV32 - RISC-V GCC Linker Script >>                                                      # */
/* # ********************************************************************************************* # */
/* # BSD 3-Clause License                                                                          # */
/* #                                                                                               # */
/* # Copyright (c) 2022, Stephan Nolting. All rights reserved.                                     # */
/* #                                                                                               # */
/* # Redistribution and use in source and binary forms, with or without modification, are          # */
/* # permitted provided that the following conditions are met:                                     # */
/* #                                                                                               # */
/* # 1. Redistributions of source code must retain the above copyright notice, this list of        # */
/* #    conditions and the following disclaimer.                                                   # */
/* #                                                                                               # */
/* # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     # */
/* #    conditions and the following disclaimer in the documentation and/or other materials        # */
/* #    provided with the distribution.                                                            # */
/* #                                                                                               # */
/* # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  # */
/* #    endorse or promote products derived from this software without specific prior written      # */
/* #    permission.                                                                                # */
/* #                                                                                               # */
/* # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   # */
/* # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               # */
/* # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    # */
/* # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     # */
/* # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE # */
/* # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    # */
/* # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     # */
/* # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  # */
/* # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            # */
/* # ********************************************************************************************* # */
/* # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting # */
/* ################################################################################################# */

/* Default linker script, for normal executables */
/* Copyright (C) 2014-2020 Free Software Foundation, Inc.
   Copying and distribution of this script, with or without modification,
   are permitted in any medium without royalty provided the copyright
   notice and this notice are preserved. */

/* modified for the NEORV32 processor by Stephan Nolting */


OUTPUT_FORMAT("elf32-littleriscv", "elf32-littleriscv", "elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(_start)
SEARCH_DIR("/opt/riscv/riscv32-unknown-elf/lib"); SEARCH_DIR("=/opt/riscv/riscv64-unknown-linux-gnu/lib"); SEARCH_DIR("=/usr/local/lib"); SEARCH_DIR("=/lib"); SEARCH_DIR("=/usr/lib");

/* **************************************************************************
 * NEORV32 memory section configuration.                                     
 * **************************************************************************
 * "ram"   : data memory (int/ext DMEM) - make sure this is sync with the HW!
 * "rom"   : instruction memory (int/ext IMEM or bootloader ROM)             
 * "iodev" : peripheral/IO devices                                           
 * ************************************************************************** */
MEMORY
{
/* section base addresses and sizes have to be a multiple of 4 bytes
 * ram section: first value of LENGTH => data memory used by bootloader (fixed!); second value of LENGTH => *physical* size of data memory
 * adapt the right-most value to match the *total physical data memory size* of your setup */

  ram  (rwx) : ORIGIN = 0x80000000, LENGTH = DEFINED(make_bootloader) ? 512 : 2*1024

/* rom and iodev sections should NOT be modified by the user at all! 
 * rom section: first value of ORIGIN/LENGTH => bootloader ROM; second value of ORIGIN/LENGTH => maximum *logical* size of instruction memory */

  rom   (rx) : ORIGIN = DEFINED(make_bootloader) ? 0xFFFF0000 : 0x00000000, LENGTH = DEFINED(make_bootloader) ? 32K : 2048M
  iodev (rw) : ORIGIN = 0xFFFFFE00, LENGTH = 512

}
/* ************************************************************************* */

SECTIONS
{
  /* start section on WORD boundary */
  . = ALIGN(4);

  /* default heap size: we can use up to 1/4 of available data memory (RAM) by default
   * WARNING! the heap will collide with the stack if allocating too much memory! */
  __heap_size = DEFINED(__heap_size) ? __heap_size : LENGTH(ram) / 4;


  /* First part of the actual executable file: actual instructions */
  .text :
  {
    PROVIDE(__text_start = .);
    PROVIDE(__textstart = .);

    KEEP(*(.text.crt0)); /* keep start-up code crt0 right at the beginning of rom */

    *(.text.unlikely .text.*_unlikely .text.unlikely.*)
    *(.text.exit .text.exit.*)
    *(.text.startup .text.startup.*)
    *(.text.hot .text.hot.*)
    *(SORT(.text.sorted.*))
    *(.text .stub .text.* .gnu.linkonce.t.*)
    /* .gnu.warning sections are handled specially by elf.em.  */
    *(.gnu.warning)

    /* finish section on WORD boundary */
    . = ALIGN(4);

    PROVIDE (__etext = .);
    PROVIDE (_etext = .);
    PROVIDE (etext = .);
  } > rom


  /* Second part of the actual executable: read-only data, placed right next to .text */
  .rodata :
  {
    /* these are a list of 32-bit pointers that point to functions
     * that are called before/after executing "main". */


    /* constructors are not supported by default. */

    /* __init_array_start = .;
     * KEEP (*(.preinit_array))
     * KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
     * KEEP (*(.rodata EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
     * __init_array_end = .;
     */


    /* main should never return, hence there is no default support for deconstructors!
     * however, if main return, the "after_main_handler" will be called by crt0, which can be used to implement
     * _minimal_ deconstructor support or to call __libc_fini_array is really really required. */

    /*
     * __fini_array_start = .;
     * KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
     * KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
     * __fini_array_end = .;
     */


    /* constant data like strings */

    *(.rodata .rodata.* .gnu.linkonce.r.*)
    *(.rodata1)


    /* finish section on WORD boundary */
    . = ALIGN(4);
  } > rom


  /* initialized read/write data, accessed in RAM, placed in ROM, copied during boot
   * not part of the final executable */
  .data :
  {
    __DATA_BEGIN__ = .;
    __SDATA_BEGIN__ = .;
    *(.sdata2 .sdata2.* .gnu.linkonce.s2.*)
    *(.data1)
    *(.data .data.* .gnu.linkonce.d.*)
    SORT(CONSTRUCTORS)

    *(.data.rel.ro.local* .gnu.linkonce.d.rel.ro.local.*) *(.data.rel.ro .data.rel.ro.* .gnu.linkonce.d.rel.ro.*)
    *(.dynamic)

    /* We want the small data sections together, so single-instruction offsets
       can access them all, and initialized data all before uninitialized, so
       we can shorten the on-disk segment size.  */

    *(.srodata.cst16) *(.srodata.cst8) *(.srodata.cst4) *(.srodata.cst2) *(.srodata .srodata.*)
    *(.sdata .sdata.* .gnu.linkonce.s.*)

    PROVIDE_HIDDEN (__tdata_start = .);
    *(.tdata .tdata.* .gnu.linkonce.td.*)


    /* finish section on WORD boundary */
    . = ALIGN(4);

    _edata = .; PROVIDE (edata = .);
    . = .;
    __DATA_END__ = .;
    __global_pointer$ = __DATA_END__ + 0x800;

  } > ram AT > rom


  /* zero/non-initialized read/write data placed in RAM - not part of the final executable */
  .bss (NOLOAD):
  {
    . = ALIGN(4);
    __BSS_START__ = .;
    *(.dynsbss)
    *(.sbss .sbss.* .gnu.linkonce.sb.*)
    *(.sbss2 .sbss2.* .gnu.linkonce.sb2.*)
    *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon)
    *(.scommon)
    *(.dynbss)
    *(.bss .bss.* .gnu.linkonce.b.*)

    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    *(COMMON)
    /* Align here to ensure that the .bss section occupies space up to
       _end.  Align after .bss to ensure correct alignment even if the
       .bss section disappears because there are no input sections.
       FIXME: Why do we need it? When there is no .bss section, we do not
       pad the .data section.  */
    . = ALIGN(. != 0 ? 32 / 8 : 1);

    . = ALIGN(4);
    __BSS_END__ = .;
    _end = .; PROVIDE (end = .);
  } > ram


  /* heap for dynamic memory allocation (use carefully!) -  not part of the final executable */
  .heap :
  {
    PROVIDE(__heap_start = .);
    . = __heap_size;
    PROVIDE(__heap_end = .);
  } > ram


  /* Yet unused */
  .jcr                : { KEEP (*(.jcr)) }
  .got                : { *(.got.plt) *(.igot.plt) *(.got) *(.igot) }  .interp         : { *(.interp) }
  .note.gnu.build-id  : { *(.note.gnu.build-id) }
  .hash               : { *(.hash) }
  .gnu.hash           : { *(.gnu.hash) }
  .dynsym             : { *(.dynsym) }
  .dynstr             : { *(.dynstr) }
  .gnu.version        : { *(.gnu.version) }
  .gnu.version_d      : { *(.gnu.version_d) }
  .gnu.version_r      : { *(.gnu.version_r) }
  .rela.init          : { *(.rela.init) }
  .rela.text          : { *(.rela.text .rela.text.* .rela.gnu.linkonce.t.*) }
  .rela.fini          : { *(.rela.fini) }
  .rela.rodata        : { *(.rela.rodata .rela.rodata.* .rela.gnu.linkonce.r.*) }
  .rela.data.rel.ro   : { *(.rela.data.rel.ro .rela.data.rel.ro.* .rela.gnu.linkonce.d.rel.ro.*) }
  .rela.data          : { *(.rela.data .rela.data.* .rela.gnu.linkonce.d.*) }
  .rela.tdata         : { *(.rela.tdata .rela.tdata.* .rela.gnu.linkonce.td.*) }
  .rela.tbss          : { *(.rela.tbss .rela.tbss.* .rela.gnu.linkonce.tb.*) }
  .rela.ctors         : { *(.rela.ctors) }
  .rela.dtors         : { *(.rela.dtors) }
  .rela.got           : { *(.rela.got) }
  .rela.sdata         : { *(.rela.sdata .rela.sdata.* .rela.gnu.linkonce.s.*) }
  .rela.sbss          : { *(.rela.sbss .rela.sbss.* .rela.gnu.linkonce.sb.*) }
  .rela.sdata2        : { *(.rela.sdata2 .rela.sdata2.* .rela.gnu.linkonce.s2.*) }
  .rela.sbss2         : { *(.rela.sbss2 .rela.sbss2.* .rela.gnu.linkonce.sb2.*) }
  .rela.bss           : { *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*) }


  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }
  .gnu.build.attributes : { *(.gnu.build.attributes .gnu.build.attributes.*) }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line .debug_line.* .debug_line_end) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes 0 : { *(.debug_pubtypes) }
  .debug_ranges   0 : { *(.debug_ranges) }
  /* DWARF Extension.  */
  .debug_macro    0 : { *(.debug_macro) }
  .debug_addr     0 : { *(.debug_addr) }
  .gnu.attributes 0 : { KEEP (*(.gnu.attributes)) }
  /DISCARD/ : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }


  /* Export symbols for neorv32 crt0 start-up code */
  PROVIDE(__crt0_imem_begin          = ORIGIN(rom));
  PROVIDE(__crt0_dmem_begin          = ORIGIN(ram));
  PROVIDE(__crt0_stack_begin         = (ORIGIN(ram) + LENGTH(ram)) - 4);
  PROVIDE(__crt0_bss_start           = __BSS_START__);
  PROVIDE(__crt0_bss_end             = __BSS_END__);
  PROVIDE(__crt0_copy_data_src_begin = __etext + SIZEOF(.rodata));
  PROVIDE(__crt0_copy_data_dst_begin = __DATA_BEGIN__);
  PROVIDE(__crt0_copy_data_dst_end   = __DATA_BEGIN__ + SIZEOF(.data));
  PROVIDE(__crt0_io_space_begin      = ORIGIN(iodev));
  PROVIDE(__crt0_io_space_end        = ORIGIN(iodev) + LENGTH(iodev));
}
