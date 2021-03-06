# The olsr.org Optimized Link-State Routing daemon (olsrd)
#
# (c) by the OLSR project
#
# See our Git repository to find out who worked on this file
# and thus is a copyright holder on it.
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
# * Neither the name of olsr.org, olsrd nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Visit http://www.olsr.org for more information.
#
# If you find this software useful feel free to make a donation
# to the project. For more information see the website or contact
# the copyright holders.
#

OLSRD_PLUGIN =	true
PLUGIN_NAME =	olsrd_nameservice
PLUGIN_VER =	0.3

TOPDIR = ../..
include $(TOPDIR)/Makefile.inc

ifeq ($(OS),win32)
default_target install clean:
	@echo "**** We use the regex library here. Does Win32 has something like this?"
else
ifeq ($(OS),android)
# On Android Google forgot to include regex engine code for Froyo version (but also there was
# no support in older versions for it) so we have here this missing code.
# http://groups.google.com/group/android-ndk/browse_thread/thread/5ea6f0650f0e3fc
OBJS +=     $(REGEX_LIB)
CFLAGS += -D__POSIX_VISIBLE
endif

default_target: $(PLUGIN_FULLNAME)

$(PLUGIN_FULLNAME): $(OBJS) version-script.txt
ifeq ($(VERBOSE),0)
		@echo "[LD] $@"
endif
		$(MAKECMDPREFIX)$(CC) $(LDFLAGS) -o $(PLUGIN_FULLNAME) $(OBJS) $(LIBS)

install:	$(PLUGIN_FULLNAME)
		$(STRIP) $(PLUGIN_FULLNAME)
		$(INSTALL_LIB)

uninstall:
		$(UNINSTALL_LIB)

clean:		$(REGEX_CLEAN)
		rm -f $(OBJS) $(SRCS:%.c=%.d) $(PLUGIN_FULLNAME)
endif
