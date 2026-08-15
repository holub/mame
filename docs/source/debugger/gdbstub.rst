.. _debugger-gdbstub:

GDB Remote Serial Protocol stub
===============================

MAME can expose a running system to GDB (or any other client of the GDB
Remote Serial Protocol) instead of showing its own debugger front-end.
Start MAME like this::

    mame <system> -debugger gdbstub -debug -debugger_port 2159

and connect from the client::

    (gdb) target remote 127.0.0.1:2159

GDB only supports the architectures it was built for, so debugging a
non-native CPU (Z80, 6502, 68000...) needs a multi-target build of the
client: on Debian/Ubuntu install the ``gdb-multiarch`` package (the
plain ``gdb`` package is x86-only and will not accept the target
description).  Raw RSP clients are unaffected.

The stub talks to the first CPU in the system.  Register reads and
writes, memory reads and writes, hardware breakpoints and watchpoints,
single-stepping, and the ``monitor`` command (which forwards to the MAME
debugger console, see :ref:`debugger`) are supported.


Session lifecycle
-----------------

The stub listens for a single TCP connection and serves one debug
session per MAME run.  This follows the remote serial protocol's
model of a single controlling debugger, and the listening socket is
closed once the first connection is accepted:

* After the client disconnects, the stub does not accept new
  connections; restart MAME for another session.
* GDB's ``detach`` resumes the emulated system and leaves MAME
  running.  The connection is closed once the reply is flushed, so the
  port is free for a new listener (e.g. another MAME instance).
* GDB's ``kill`` (sent automatically in some batch/exit paths) makes
  MAME exit.


Protocol capabilities
---------------------

The stub advertises and implements:

* ``PacketSize``
* ``qXfer:features:read:target.xml`` -- a dynamically generated target
  description
* ``qOffsets`` -- answered ``Text=0;Data=0;Bss=0``; the CPU address space
  is not relocated, so a symbol file loaded with ``file`` or
  ``add-symbol-file`` is used as-is
* ``qRcmd`` -- the argument is decoded and executed as a MAME debugger
  console command; console output is returned
* registers ``g``/``G``/``p``/``P`` (only after the target description
  has been read), memory ``m``/``M``, breakpoints/watchpoints ``Z``/``z``
  (hardware), step ``s``, continue ``c``

Symbol files
------------

The stub reports all-zero section offsets (see ``qOffsets`` above), so
a symbol file is used as-is: link or wrap it at the addresses the code
actually runs at.  Left unanswered, ``qOffsets`` makes GDB assume the
sections were relocated, resolve symbols at wrong addresses, and place
breakpoints that never hit.

The machine type in the file header should not be z80: GDB (verified
with 17.2) crashes on a symbol lookup in a z80-mach
ELF.  Use any other machine type as a carrier -- the architecture
declared in the target description takes over once connected.  A raw
binary can be turned into such a symbol file with ``objcopy``::

    objcopy -I binary -O elf32-big --binary-architecture=m68k \
        --add-symbol boot_continue=0x69,global,function \
        boot.bin boot.elf

For banked code, several pieces of code can share one address window.
Name-based commands (``break``, ``list``) pick the right piece by
symbol name; for shared addresses use the exported MMU registers, e.g.
``break *0x8000 if $mmu4 == 0x21`` stops only when page 0x21 is mapped
at 0x8000-0x9fff.


I/O ports
---------

GDB's memory commands only reach the program address space; the RSP
protocol has no notion of an I/O space.  Use the ``monitor`` pass-through
with MAME's debugger expressions instead: the memory operator
``<device>.<space><size>@<address>`` reads or writes any address space.
The space letter is ``p`` for program, ``d`` for data and ``i`` for
I/O, the size letter is ``b``, ``w``, ``d`` or ``q``, and the device
tag is optional (the CPU currently visible in the debugger is used).
``@`` performs a debugger-style access with side effects suppressed;
``!`` performs the access like real hardware.  This matters in both
directions: a suppressed read leaves device state alone (a FIFO or
read-ahead is not advanced), and a suppressed write can be dropped
silently by the device.  Use ``!`` when a write must take effect.
Use ``monitor print`` to read and ``monitor do`` to write::

    (gdb) monitor print ib@0xfe
    BF
    (gdb) monitor do ib!0x243b = 0x11
    (gdb) monitor print ib@0x243b
    11
    (gdb) monitor do ib!0x243b = 0xa
    (gdb) monitor print ib@0x243b
    A
    (gdb) monitor print maincpu.ib@0xfe
    BF

The example is from the ZX Spectrum Next: port 0xfe reads the keyboard
(BF means no keys pressed), and port 0x243b is the NextReg select
register, which reads back what was written.

Debugger commands from GDB
--------------------------

``monitor`` is not limited to ``print`` and ``do``: every command is
passed to the debugger console command parser, and its output comes
back as console output.  This reaches functionality GDB has no packet
for -- savestates, memory files, tracing, or resetting the machine::

    (gdb) monitor statesave checkpoint
    State save attempted.  Please refer to window message popup for results.
    (gdb) monitor stateload checkpoint
    State load attempted.  Please refer to window message popup for results.
    (gdb) monitor save "dump.bin",0x9000,16
    Data saved successfully

``statesave``/``stateload`` (aliases ``ss``/``sl``) save to and load
from ``sta/<driver>/<name>``; the popup message is generic console
wording, the operation itself is synchronous.  ``save`` writes a raw
binary image of an address range (``load`` reads one back, ``dump``
formats a hex dump); commands taking a filename need the quoted,
comma-separated form shown above.  ``monitor help`` lists the command
table, ``monitor help <command>`` a single command's usage.

The boundary: RSP is request/response with no asynchronous
notifications beyond stop replies, so there is no equivalent to the
Lua plugins' callbacks (per-frame hooks, timers, event handlers).
Automation driven by the target belongs on the GDB side -- its Python
API can issue ``monitor`` commands on breakpoint hits -- or in MAME's
Lua plugins.




Target description and register numbering
-----------------------------------------

The target description contains one feature per CPU core
(e.g. ``org.gnu.gdb.z80.cpu``) followed by one ``mame.<path>`` feature
per additional device that exposes debugger state, such as the driver
state class.  ``<path>`` is the device's absolute path: its full device
tag with the ``:`` separators changed to ``/``.  Every visible,
non-divider state entry of those devices is exported as a register.

The description does real work, and serving it is not optional: without
it GDB falls back to per-architecture built-in guesses about the
register layout, which match neither the names nor the sizes the cores
actually expose.  The CPU feature must carry the exact name and the
complete register set GDB's architecture support looks for --
``org.gnu.gdb.z80.cpu`` with all thirteen registers, or GDB reports an
unrecognized architecture -- which is why it comes first, and the extra
device registers ride along as features after it.

The register list is built at connect time from the state entries the
devices actually implement: an entry that does not exist is skipped,
and the registers after it shift down in numbering.  Until GDB has
fetched the description the stub leaves ``g``/``G``/``p``/``P``
unanswered -- GDB probes registers before reading it, and regnums it
does not know yet would be misdecoded.

The same path is also the name of the register group holding the
device's registers: the driver state class is the root device, so its
registers group under ``/``, a directly attached device groups under
``/dma``, a nested one under ``/exp/spi/sd``.  ``info registers
<group>`` lists a single device's registers without enumerating the
whole machine, and ``maintenance print reggroups`` lists the available
groups.  The exported registers are integers, so they also show up in
the plain ``info registers`` and ``info all-registers`` listings.

For the ZX Spectrum Next (``tbblue``) this yields:

* regnums ``0``-``0x0C``: the thirteen Z80 registers
  (``af``, ``bc``, ``de``, ``hl``, ``af'``, ``bc'``, ``de'``, ``hl'``,
  ``ix``, ``iy``, ``sp``, ``pc``, ``ir``).  ``ir`` is a composed view
  of the I and R registers (``I<<8 | R``); writing it updates both.
* regnums ``0x0D``-``0x14``: ``mmu0``-``mmu7`` (each MMU register maps
  an 8 KB page -- ``mmu0`` is 0x0000-0x1fff, ``mmu2`` 0x4000-0x5fff,
  ``mmu4`` 0x8000-0x9fff, \.\.\.)
* regnums ``0x15``-``0x114``: ``nr00``-``nrff``, the NextReg space.
  Reads and writes go through the same dispatch as the hardware
  register select/data ports, so debugger writes trigger the real
  register side effects (bank switches, palette updates, resets\.\.\.).

The order of the first twelve Z80 registers (``af``\.\.\.``pc``) is a de
facto wire contract relied upon by raw RSP clients and must not be
changed.


Example GDB session (ZX Spectrum Next)
--------------------------------------

.. code-block:: text

    $ gdb-multiarch -q boot.elf                # optional symbol file
    (gdb) target remote 127.0.0.1:2159
    0x00000000 in ?? ()

    (gdb) info registers af hl sp pc ix iy ir mmu4 mmu5 nr00 nr01 nr0e
    af             0x40                64
    hl             0x0                 0
    sp             0x0                 0x0
    pc             0x0                 0x0
    ix             0xffff              -1
    iy             0xffff              -1
    ir             0x0                 0
    mmu4           0x4                 4 '\004'
    mmu5           0x5                 5 '\005'
    nr00           0x8                 8 '\b'
    nr01           0x32                50 '2'
    nr0e           0x4                 4 '\004'

    (gdb) break boot_continue
    Breakpoint 1 at 0x69
    (gdb) continue
    Breakpoint 1, 0x00000069 in boot_continue ()
    (gdb) x/2i $pc
    => 0x69 <boot_continue>:	ld sp,0x6000
       0x6c <boot_continue+3>:	ld hl,0x1fb2

    (gdb) python gdb.selected_inferior().write_memory(0x9000, b"\xde\xad\xbe\xef")
    (gdb) x/wx 0x9000
    0x9000:	0xefbeadde
    (gdb) x/hx 0x9000
    0x9000:	0xadde
    (gdb) x/hx 0x9002
    0x9002:	0xefbe

    (gdb) monitor help trace
      trace {<filename>|off}[,<CPU>[,[noloop|logerror][,<action>]]]
      \.\.\.

    (gdb) detach


Example raw RSP session
-----------------------

Switching the RAM bank visible at 0x8000-0x9fff by writing the MMU
register (through both the NextReg port dispatch ``nr54`` and the
``mmu4`` state entry):

.. code-block:: text

    -> p11                      <- 04            ; mmu4
    -> m8000,8                  <- ccd718893d7ece47  (bank 0x04)
    -> P15+54=21                <- OK            ; nr54 := 0x21
    -> m8000,8                  <- c323bd9e1645a4e6  (bank 0x21)
    -> P11=33                   <- OK            ; mmu4 := 0x33
    -> m8000,8                  <- 3130dcb5b7f51740  (bank 0x33)

Composed register and hostile input:

.. code-block:: text

    -> P0c=0304                 <- OK            ; ir := 0x0304
    -> p0c                      <- 0304          ; I=3, R=4
    -> qXfer:features:read:target.xml:fffff,fff
    <- l                                        ; clamped, empty reply

The last line is GDB's probe past the end of the target description:
reads beyond the document are clamped to its length and answered as an
empty final chunk.  Taking such an offset literally would run off the
generated XML buffer inside the stub.


Known limitation: GDB z80 frame unwinding
-----------------------------------------

GDB (verified with 17.2) with a Z80 target may appear to hang on
``set $reg`` and ``set {type}addr = value`` when the emulated system has
no usable stack, e.g. while halted at reset.  GDB's classic frame
unwinder rebuilds the frame chain for the write and marches through
memory following garbage "return addresses" -- two stack bytes per frame,
wrapping the 16-bit address space indefinitely.  Loading symbols does
not help.  The ``P``/``G``/``M`` packets themselves are answered
immediately; ``p`` reads, breakpoints, ``continue``, GDB's Python
``inferior.write_memory()``, and raw RSP clients are unaffected.

Workarounds for writing from GDB:

* memory: ``python gdb.selected_inferior().write_memory(addr, bytes)``
* registers: use a raw RSP client, or MAME's own debugger front-end
