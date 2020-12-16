#
# File:
#    Makefile
#
# Description:
#    Makefile for the coda primary and secondary readout lists
#    running on an Intel-based controller running Linux
#
#
# Uncomment DEBUG line for debugging info ( -g and -Wall )
DEBUG=1
QUIET=1
#
ifeq ($(QUIET),1)
        Q = @
else
        Q =
endif

# Plug in your primary readout lists here.. CRL are found automatically
VMEROL			= tdc1190_list.so c792_list.so ti_master_list.so bbhodo_list.so
# Add shared library dependencies here.  (jvme, ti, are already included)
ROLLIBS			= -lc1190 -lc792 -lsd -lts

ifdef CODA_VME
INC_CODA_VME	= -isystem${CODA_VME}/include
endif

ifdef CODA_VME_LIB
LIB_CODA_VME	= -L${CODA_VME_LIB}
endif

LINUXVME_LIB	?= .
LINUXVME_INC	?= .

# DEFs for compiling primary readout lists
CC			= gcc
AR                      = ar
RANLIB                  = ranlib
ifdef DEBUG
CFLAGS			= -Wall -g
else
CFLAGS			= -O3
endif
CFLAGS			+= -DLINUX -DDAYTIME=\""`date`"\"

INCS			= -I. -I${LINUXVME_INC} ${INC_CODA_VME} \
				-isystem${CODA}/common/include
LIBS			= -L. -L${LINUXVME_LIB} ${LIB_CODA_VME} -DJLAB \
				-lrt -lpthread -ljvme -lti $(ROLLIBS)

# DEFs for compiling CODA readout lists
CCRL			= ${CODA_BIN}/ccrl
CODA_INCS		= -I. -I${LINUXVME_INC} ${INC_CODA_VME} -isystem${CODA}/common/include
CODA_LIBDIRS            = -L.
CODA_LIBS		=
CODA_DEFS		= -DLINUX -DDAYTIME=\""`date`"\"
ifdef DEBUG
CODA_CFLAGS		= -Wall -g
else
CODA_CFLAGS		= -O
endif
CODA_CFLAGS		+= -w -fpic -shared ${CODA_INCS} ${CODA_LIBDIRS} \
			  ${CODA_LIBS} ${CODA_DEFS}
CRLFILES		= $(wildcard *.crl)
CFILES			= $(CRLFILES:.crl=.c)
SOBJS			= $(CRLFILES:.crl=.so)
DEPS			= $(VMEROL:%.so=%.d)
DEPS			+= $(CFILES:%.c=%.d)


all:  $(VMEROL) $(SOBJS)

%.c: %.crl
	@echo " CCRL   $@"
	${Q}${CCRL} $<

event_list.so: event_list.c
	@echo " CC     $@"
	${Q}${CC} ${CODA_CFLAGS} -o $@ $<

test_list_v3.so: test_list_v3.c
	@echo " CC     $@"
	${Q}${CC} ${CODA_CFLAGS} -o $@ $<

%.so: %.c
	@echo " CC     $@"
	${Q}$(CC) -fpic -shared  $(CFLAGS) $(INCS) $(LIBS) \
		-DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<

bbhodo_slave_list.so: bbhodo_list.c
	@echo " CC     $@"
	${Q}$(CC) -fpic -shared  $(CFLAGS) $(INCS) $(LIBS) -DTI_SLAVE -DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<

bbhodo_list.so: bbhodo_list.c
	@echo " CC     $@"
	${Q}$(CC) -fpic -shared  $(CFLAGS) $(INCS) $(LIBS) -DTI_MASTER -DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<

blah_slave_list.so: blah_list.c
	@echo " CC     $@"
	${Q}$(CC) -fpic -shared  $(CFLAGS) $(INCS) $(LIBS) -DTI_SLAVE -DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<

blah_list.so: blah_list.c
	@echo " CC     $@"
	${Q}$(CC) -fpic -shared  $(CFLAGS) $(INCS) $(LIBS) -DTI_MASTER -DINIT_NAME=$(@:.so=__init) -DINIT_NAME_POLL=$(@:.so=__poll) -o $@ $<

clean distclean:
	${Q}rm -f  $(VMEROL) $(SOBJS) $(CFILES) *~ $(DEPS) $(DEPS) *.d.*

%.d: %.c
	@echo " DEP    $@"
	${Q}set -e; rm -f $@; \
	$(CC) -MM -shared $(INCS) $(CFLAGS) -DINIT_NAME=$(@:.so=__init) \
		-DINIT_NAME_POLL=$(@:.so=__poll) $< > $@.$$$$; \
	sed 's,\($*\)\.o[ :]*,\1.so $@ : ,g' < $@.$$$$ > $@; \
	rm -f $@.$$$$

-include $(DEPS)

.PHONY: all
