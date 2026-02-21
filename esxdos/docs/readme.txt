1. About ESXDOS
===============

ESXDOS aims to be the ultimate firmware for the DivIDE interface. Here's a list of current features:

* Supports HDD/CDROM/ZIP/LS120 devices
* Device and filesystem abstraction layer
* Full FAT16/FAT32 read/write support (no 64K clusters, no extended partitions, no LFN).
* Provides extended BASIC commands
* BASIC files integration using +3DOS headers for FAT filesystems
* Support for seamless IM2 loading/saving, from BASIC and machine code
* System commands loaded from /BIN dir of system drive
* TAPE emulator supports reading/writing from/to TAP files. TAP attaching functions are available to external programs.
* POSIX-based API usable by .commands, external programs and NMI.SYS. Functions available on rst $08:
  open, read, write, close, opendir, readdir, seek, sync, fstat, getcwd, chdir, unlink...
* Possibility of getting absolute LBA sector and device on an opened file (for direct I/O)
* Kernel loads modules (.KO files) on demand
* NMI.SYS support (NMI system is independent, ESXDOS kernel just provides services)

On the pipeline:

1. Virtual drives
2. TR-DOS emulator
3. LFN support

2. How to setup your system drive for ESXDOS
============================================

Make sure you copy SYS and BIN directories to the root of your system drive. The contents should be as follows:

/SYS:	bdir.ko, bfile.ko, errmsg.ko, esxdos.sys, nmi.sys, tape.ko

/BIN:	cd, chmod, cp, dskprobe, dumpmem, file, hexdump, ls, lstap, mkdir, more,
		mv, partinfo, playpt3, playsqt, playstc, snapload, tapein, tapeout

You should also create /TMP dir as it's the location used to store temporary files (if/when needed).

3. How to flash ESXDOS
======================

Copy ESXDOS080.TAP to your system drive and load/run it from ESXDOS 0.7.x or some other firmware. Follow the on-screen instructions. ESXDOS080.BIN is the raw ROM image to be used with burners/emulators.

4. Checking your setup
======================

When cold booting ESXDOS the BIOS screen will be shown. Make sure ESXDOS.SYS and NMI.SYS load OK. After a cold boot you can type RUN to check if system loaded correctly - you should get either "O.K. ESXDOS" or "No SYSTEM" message.

If you want to cold boot/reinit devices, press reset while holding space.

5. NMI Browser
==============

ESXDOS now has the functionality that users most requested - an NMI browser/loader coded by ub880d (ub880d at zxmail dot org). Here are some instructions from him:

Usage:

'r' - reset (warm)
's' - create snapshot (autoincrement names start with snap0000.sna on initialization of esxdos. If you get ERROR 18 it's because file already exists, try again)
up,down - move cursor on page
left,right - change page
'1' - go to parent directory (chdir to '..')
'v' - show screen from .scr and .sna (or files with ZX header of type 'CODE')
'l' - attach tape file to input slot
enter - run sna, z80, files with ZX header of type 'basic', view screen (same as 'v' except for sna files), attach tap file to input slot and soft reset with autoload
space - exit from nmi

TODO (in order of importance - but order could be changed if no enough free space for particular step):
1) lazy scan dir
2) 2 level paging (no 22 files on 32 pages = 704 files, but 22 files on 8 pages on 8 pagesets = 1408 files)
3) user input filename when save snapshot
4) nicer error reporting (for example put red background to status line)

(c) 2011-2012 ub880d, except for core 'save snapshot' code. for snapshot saving used example of create snapshot docs/nmi_sys.asm, (c) 2012 lordcoxis

6. System commands
==================

Files located in the /BIN directory are system commands and can be executed from BASIC by typing ".command <args>". Most commands are self-explanatory or show online help when run without arguments (or with -h parameter).

Examples:

.ls						(show dir listing)
.cd	games 				(change directory)
.tapein esxdos.tap		(attach .tap file for reading)
.tapeout newdemo.tap	(attach .tap file for writing)
.chmod +h esxdos.sys	(set/unset attributes on file/dir)
.mkdir newdir			(create new directory)
.mv oldname newname		(rename/move file/dir)
.cp source target		(copy file)
.more textfile			(show contents of text file)

7. BASIC commands
=================

ESXDOS extends some BASIC commands functionality. Please note the following:

- All commands support a <drive> parameter, which can be "*" for current drive or you can specify another one (ie hd1).
- You can use a BASIC variable instead of a filename: just put a ";" between the drive and variable.
- In case of FAT filesystems, all files created by ESXDOS will have a +3DOS header.

Drive Naming Convention:

Drives are named according to their type and partition number. So for the first hard drive and partition, it would be named hd0. You can see a list of drive names when ESXDOS runs it's initial BIOS drive detection.

Filenames:

Altough ESXDOS itself supports long filenames, the current FAT driver does not. So, for now, filenames are limited to the original 8.3 format.

Here's a list of all commands with examples:

CAT:
----

Displays a simple directory listing when used without parameters. When used with drive parameter will show extended listing, with BASIC header and free space info.

Form:

CAT [<drive>]

Examples:

CAT
CAT *
CAT hd1

GOTO:
-----

Show current drive/directory or change drive/directory.

Forms:

GOTO (shows current drive/directory)
GOTO [drive] [path] (changes current drive/directory)

Examples:

GOTO hd1
GOTO "new/path"
GOTO hd0 "/new/path"

LOAD:
-----

LOAD a file from disk. Headerless files will be loaded as CODE with START=32768.

Forms:

LOAD <drive> "path/to/filename"
LOAD <drive> "path/to/filename" CODE [<START>] [<LENGTH>]
LOAD <drive> "path/to/filename" SCREEN$

Examples:

LOAD * "filename"
LOAD * "filename" CODE 32768,16384
LOAD * "filename" SCREEN$

MERGE:
------

MERGE a file from disk.

Form:

MERGE <drive> "path/to/filename"

Examples:

MERGE * "filename"

VERIFY:
-------

VERIFY a file from disk (compare it against RAM contents). Headerless files will be verified as CODE with START=32768.

Forms:

VERIFY <drive> "path/to/filename"
VERIFY <drive> "path/to/filename" CODE [<START>] [<LENGTH>]
VERIFY <drive> "path/to/filename" SCREEN$

Examples:

VERIFY * "filename"
VERIFY * "filename" CODE 32768,16384
VERIFY * "filename" SCREEN$

SAVE:
-----

SAVE a file to disk. If the file already exists, confirmation will be requested.

Forms:

SAVE <drive> "path/to/filename" [LINE]
SAVE <drive> "path/to/filename" CODE <START> <LENGTH>
SAVE <drive> "path/to/filename" SCREEN$

Examples:

SAVE * "filename" LINE 10
SAVE * "filename" CODE 32768,16384
SAVE * "filename" SCREEN$

ERASE:
------

Erase a file or an empty directory. If the file/dir is in use an error message will be displayed ("Acess Denied").

Form:

ERASE [<drive>] "path/to/file_or_dir"

Examples:

ERASE "filename"
ERASE "dirname"
ERASE hd1 "somefile"