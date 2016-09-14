include Makefile.inc

TOPDIR = ../..
include $(TOPDIR)/Makefile.inc

NMEALIB_PATH = $(TOPDIR)/lib/gpsreceiver/nmealib
NMEALIB_NAME := nmea
# INCDIRS = $(NMEALIB_PATH)/include
# CFLAGS=-I$(INCDIRS)

NMEALIB_STATICALLY_LINKED ?= 1
CFLAGS += -D_GNU_SOURCE


RESOURCESDIR = ./resources
NMEALIB_LIB = $(NMEALIB_PATH)/lib
NMEALIB_LIB_STATIC = $(NMEALIB_LIB)/lib$(NMEALIB_NAME).a

#CFLAGS += -I $(NMEALIB_INC)
ifeq ($(NMEALIB_STATICALLY_LINKED),)
LIBS += -L $(NMEALIB_LIB) -l$(NMEALIB_NAME) -lm
else
# libnmea is statically linked
LIBS += -lm
endif

CPPFLAGS += -I$(NMEALIB_PATH)/include

SUPPORTED = 0
ifeq ($(OS),linux)
SUPPORTED = 1
endif
ifeq ($(OS),android)
SUPPORTED = 1
endif


ifeq ($(SUPPORTED),0)

.PHONY: all default_target install clean

all: default_target

default_target install clean:
	@echo "*** $(PLUGIN_NAME) plugin not supported on $(OS), sorry!"

else

GIT_SHA = $(shell git describe 2> /dev/null || echo "Unknown")
CFLAGS += -DGIT_SHA=\"$(GIT_SHA)\" -DPLUGIN_VER=\"$(PLUGIN_VER)\"

.PHONY: all default_target install uninstall clean doc doc-clean nmealib

all: default_target

default_target: nmealib $(PLUGIN_FULLNAME)

# repeat from toplevel Makefile.inc, and adjust: we need the library includes
%.d: %.c
	$(MAKECMDPREFIX)$(filter-out $(CCACHE),$(CC)) -M $(strip $(CPPFLAGS)) "$<" | sed -e '1s/\($(call quote,$(*F))\.o\)[ :]*/$(call quote,$(*D)/\1 $@: Makefile $(TOPDIR)$(if $(TOPDIR),/)Makefile.inc) /g' >"$@"
#	$(MAKECMDPREFIX)$(filter-out $(CCACHE),$(CC)) -M $(strip $(CPPFLAGS)) -I $(NMEALIB_INC) "$<" | sed -e '1s/\($(call quote,$(*F))\.o\)[ :]*/$(call quote,$(*D)/\1 $@: Makefile $(TOPDIR)$(if $(TOPDIR),/)Makefile.inc) /g' >"$@"

$(PLUGIN_FULLNAME): $(OBJS) version-script.txt
ifeq ($(NMEALIB_STATICALLY_LINKED),)
ifeq ($(VERBOSE),0)
	@echo "[LD] $@ (nmealib dynamically linked)"
endif
	$(MAKECMDPREFIX)$(CC) $(LDFLAGS) -o $(PLUGIN_FULLNAME) $(OBJS) $(LIBS)
else
ifeq ($(VERBOSE),0)
	@echo "[LD] $@ (nmealib statically linked)"
endif
	$(MAKECMDPREFIX)$(CC) $(LDFLAGS) -o $(PLUGIN_FULLNAME) $(OBJS) $(NMEALIB_LIB_STATIC) $(LIBS)
endif

install: all
	$(MAKECMDPREFIX)$(MAKE) -C "$(NMEALIB_PATH)" DESTDIR="$(DESTDIR)" install
	$(INSTALL_LIB)
	$(STRIP) "$(LIBDIR)/$(PLUGIN_FULLNAME)"

uninstall:
	$(MAKECMDPREFIX)$(MAKE) -C "$(NMEALIB_PATH)" DESTDIR="$(DESTDIR)" uninstall
	rm -f "$(LIBDIR)/lib$(PLUGIN_NAME).so" "$(LIBDIR)/$(PLUGIN_NAME)"
	$(UNINSTALL_LIB)

clean:
ifeq ($(VERBOSE),0)
	@echo "[$@]"
endif
	$(MAKECMDPREFIX)rm -f $(OBJS) $(SRCS:%.c=%.d) "$(PLUGIN_FULLNAME)"
	$(MAKECMDPREFIX)$(MAKE) -C doc clean
	$(MAKECMDPREFIX)$(MAKE) -C "$(NMEALIB_PATH)" clean

doc:
	$(MAKECMDPREFIX)$(MAKE) -C doc all
	$(MAKECMDPREFIX)$(MAKE) -C "$(NMEALIB_PATH)" doc

doc-clean:
	$(MAKECMDPREFIX)$(MAKE) -C doc clean
	$(MAKECMDPREFIX)$(MAKE) -C "$(NMEALIB_PATH)" doc clean

nmealib:
	$(MAKECMDPREFIX)$(MAKE) -C "$(NMEALIB_PATH)" all

endif
