# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2003 Pascal Brisset, Antoine Drouin
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

# Quiet compilation
Q=@

OCAMLCFLAGS=-thread -ccopt -fPIC

include ../../Makefile.ocaml
OCAMLPATH:=$(shell echo $(PAPARAZZI_SRC)/sw/ground_segment/cockpit/lib:$(OCAMLPATH))
export OCAMLPATH

ifneq ($(USE_LABELGTK),lablgtk2)
all :
	@echo Skipping legacy GCS build \(missing lablgtk2\)

opt :
	@echo Skipping legacy GCS.opt build \(missing lablgtk2\)

else
INCLUDES=
PKG = -package lablgtk2,pprzlink,gcslib
LINKPKG = $(PKG) -linkpkg -dllpath-pkg lablgtk2,pprzlink,gcslib,pprz

LABLGTK2INIT = $(shell ocamlfind query -p-format lablgtk2.init 2>/dev/null)
ifeq ($(LABLGTK2INIT),)
LABLGTK2INIT = $(shell ocamlfind query -p-format lablgtk2.auto-init 2>/dev/null)
endif

LABLGTK2GLADE = $(shell ocamlfind query -p-format lablgtk2.glade 2>/dev/null)
LABLGTK2CANVAS = $(shell ocamlfind query -p-format lablgtk2-gnome.gnomecanvas 2>/dev/null)

ML= gtk_setting_time.ml gtk_strip.ml horizon.ml strip.ml gtk_save_settings.ml saveSettings.ml page_settings.ml pages.ml speech.ml plugin.ml sectors.ml map2d.ml editFP.ml intruders.ml shapes.ml live.ml particules.ml papgets.ml gcs.ml
MAIN=gcs
CMO=$(ML:.ml=.cmo)
CMX=$(ML:.ml=.cmx)

# only compile it lablgtk2 is installed
ifeq ($(LABLGTK2INIT),)
all :
	@echo Skipping legacy GCS build \(missing lablgtk2.init\)

opt :
	@echo Skipping legacy GCS.opt build \(missing lablgtk2.init\)

else

ifeq ($(LABLGTK2GLADE),)

all :
	@echo Skipping legacy GCS build \(missing lablgtk2.glade\)

opt :
	@echo Skipping legacy GCS.opt build \(missing lablgtk2.glade\)

else

ifeq ($(LABLGTK2CANVAS),)

all :
	@echo Skipping legacy GCS build \(missing lablgtk2-gnome.gnomecanvas\)

opt :
	@echo Skipping legacy GCS.opt build \(missing lablgtk2-gnome.gnomecanvas\)

else
all : $(MAIN)

opt : $(MAIN).opt

endif
endif
endif
endif

gcslib :
	$(Q)$(MAKE) -C lib

$(MAIN) : $(CMO) $(LIBPPRZCMA) $(LIBPPRZLINKCMA) $(XLIBPPRZCMA) gcslib
	@echo OL $@
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(LINKPKG) $(CMO) -o $@

$(MAIN).opt : $(CMX) $(LIBPPRZCMXA) $(LIBPPRZLINKCMXA) $(XLIBPPRZCMXA) gcslib
	@echo OOL $@
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) -package pprz.xlib,$(LABLGTK2INIT) -linkpkg $(CMX) -o $@

%.cmo: %.ml
	@echo OC $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(PKG) -c $<
%.cmi: %.mli
	@echo OCI $<
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(PKG) -c $<
%.cmx: %.ml
	@echo OOC $<
	$(Q)$(OCAMLOPT) $(OCAMLCFLAGS) $(INCLUDES) $(PKG) -c $<

saveSettings.cmo : gtk_save_settings.cmo
saveSettings.cmx: gtk_save_settings.cmx

gtk_strip.ml : gcs.glade
	@echo GLADE $@
	$(Q)lablgladecc2 -root eventbox_strip -hide-default $< | grep -B 1000000 "  end" > $@

gtk_setting_time.ml : gcs.glade
	@echo GLADE $@
	$(Q)lablgladecc2 -root setting_time -hide-default $< | grep -B 1000000 "  end" > $@

gtk_save_settings.ml : gcs.glade
	@echo GLADE $@
	$(Q)lablgladecc2 -root save_settings -hide-default $< | grep -B 1000000 "  end" > $@

strip.cmo : gtk_strip.cmo gtk_setting_time.cmo

compass : compass.ml
	@echo OL $@
	$(Q)$(OCAMLC) $(OCAMLCFLAGS) $(INCLUDES) $(LINKPKG) gtkInit.cmo $^ -o $@


clean:
	$(Q)rm -f *~* *.cm* *.o *.out *.opt map2d gcs .depend gtk_strip.ml gtk_setting_time.ml gtk_save_settings.ml compass ant_track ant_track_pmm ant_track_pmm_gtk3 actuators
	$(MAKE) -C lib clean

.PHONY: all opt clean


CC = gcc
CFLAGS=-g -O2 -Wall $(shell pkg-config gtk+-2.0 --cflags) -fPIC
LDFLAGS=$(shell pkg-config gtk+-2.0 --libs) -s -lglibivy -lm -lpcre


ant_track : ant_track.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)

ant_track_pmm : ant_track_pmm.c
	$(CC) -g -O2 -Wall $(shell pkg-config gtk+-3.0 --cflags) -fPIC -g -o $@ $^ $(shell pkg-config gtk+-3.0 --libs) -s -lglibivy -lm -lpcre -rdynamic

actuators : actuators.c
	$(CC) $(CFLAGS) -g -o $@ $^ $(LDFLAGS)


#
# Dependencies
#

.depend: Makefile
	@echo DEPEND $@
	$(Q)$(OCAMLDEP) -I $(LIBPPRZDIR) -I lib $(ML) lib/*.mli *.mli > .depend

ifneq ($(MAKECMDGOALS),clean)
-include .depend
endif
