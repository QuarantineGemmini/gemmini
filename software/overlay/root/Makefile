#=============================================================================
# variables
#=============================================================================
prefix				 := /usr/local
abs_top_srcdir := /home/centos/chipyard/generators/gemmini/software/gemmini-rocc-tests/build/..
XLEN           := 64
ROCC 					 := examples
CATEGORIES     := bareMetalC ee290 mlps imagenet 
#CATEGORIES     := bareMetalC ee290

#=============================================================================
# targets
#=============================================================================
.PHONY: all $(CATEGORIES)
all: $(CATEGORIES)

$(CATEGORIES):
	mkdir -p $@
	make -C $@	\
		-f $(abs_top_srcdir)/header.mk \
		TARGET_MAKEFILE=$(abs_top_srcdir)/$@/Makefile \
		abs_top_srcdir=$(abs_top_srcdir) \
	 	src_dir=$(abs_top_srcdir)/$@ \
	 	XLEN=$(XLEN) \
	 	PREFIX=$(ROCC)-$@


