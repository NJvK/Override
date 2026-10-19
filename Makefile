################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=1
LIBNAME:=LemLib
VERSION:=0.5.6

# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line keeps our robot code (every file directly in src/, not the LemLib code in src/lemlib/)
# out of the LemLib library. It then goes in the hot package, which starts after the PROS kernel is
# running. In the library (cold package) its motors/sensors/controller get set up before the kernel
# has started, and driver control doesn't respond.
EXCLUDE_SRC_FROM_LIB+=$(foreach ext,$(CEXTS) $(CXXEXTS),$(wildcard $(SRCDIR)/*.$(ext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the the include directory get exported

TEMPLATE_FILES=$(INCDIR)/lemlib/*.hpp $(INCDIR)/lemlib/logger/*.hpp $(INCDIR)/lemlib/chassis/*.hpp $(INCDIR)/fmt/*.h $(INCDIR)/fmt/LICENSE $(FWDIR)/hot-cold-asset.mk $(ROOT)/static/example.txt $(INCDIR)/lemlib/LICENSE $(INCDIR)/lemlib/README.md $(INCDIR)/lemlib/VERSION

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
