#!/bin/sh
git checkout -- eurasia/eurasiacon/freedesktop/X.Org/X11R7.4/pvr_video/Makefile.in
git checkout -- eurasia/eurasiacon/freedesktop/X.Org/X11R7.4/pvr_video/autom4te.cache/requests
git checkout -- eurasia/eurasiacon/freedesktop/X.Org/X11R7.4/pvr_video/variant/generic_sgx/Makefile.in
git checkout -- eurasia/tools/intern/useasm/lex.yy.c
rm -rf eurasia/eurasiacon/freedesktop/X.Org/X11R7.4/caches/
rm -f eurasia/eurasiacon/freedesktop/X.Org/X11R7.4/pvr_video/autom4te.cache/output.3
rm -f eurasia/eurasiacon/freedesktop/X.Org/X11R7.4/pvr_video/autom4te.cache/traces.3

