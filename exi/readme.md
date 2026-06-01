
This directory contains the EXI decoder/encoder, and
a project specific interface file (projectExiConnecton.c / .h).

The exi decoder/encoder files are taken from
https://github.com/Martin-P/OpenV2G
which is a mirror of
openv2g.sourceforge.net/

Last version fetched:
https://sourceforge.net/p/openv2g/code/126
0.9.6

Changes from original:

dinEXIDatatypes.h
#define dinServiceParameterListType_ParameterSet_ARRAY_SIZE 1 // originally 5 but probably reduced to save stack

DIN: DIN SPEC 70121 					Still widely supported
ISO1: Early ISO 15118 implementation	Legacy
ISO2: ISO 15118-2 						Main production standard
ISO20: ISO 15118-20 					Newest generation

