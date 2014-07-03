# Makefile : builds libbreezyslam.so C++ library
#
# Copyright (C) Simon D. Levy 2014
#
# This code is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
#
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License 
#  along with this code.  If not, see <http:#www.gnu.org/licenses/>.
#  You should also have received a copy of the Parrot Parrot AR.Drone 
#  Development License and Parrot AR.Drone copyright notice and disclaimer 
#  and If not, see 
#   <https:#projects.ardrone.org/attachments/277/ParrotLicense.txt> 
# and
#   <https:#projects.ardrone.org/attachments/278/ParrotCopyrightAndDisclaimer.txt>.

# Where you want to put the library
# LIBDIR = /usr/local/lib
LIBDIR = /home/michael/sketchbook/libraries

ifeq ("$(shell uname)","Darwin")
  LIBEXT = dylib
else ifeq ("$(shell uname)","Linux")
  CFLAGS = -fPIC
  LIBEXT = so
else
  LIBEXT = dll
endif

all: libbreezyslam.$(LIBEXT)

test: breezytest
	./breezytest

libbreezyslam.$(LIBEXT): cppcoreslam.o coreslam.o rmhc_filter.o ziggrand.o
	g++ -O3 -shared cppcoreslam.o coreslam.o rmhc_filter.o ziggrand.o \
          -o libbreezyslam.$(LIBEXT) -lm

cppcoreslam.o: cppcoreslam.cpp breezyslam.hpp ../coreslam//coreslam.h
	g++ -O3 -I../coreslam -c -Wall $(CFLAGS) cppcoreslam.cpp

coreslam.o: ../coreslam/coreslam.c ../coreslam/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) ../coreslam/coreslam.c

rmhc_filter.o: ../coreslam/rmhc_filter.c ../coreslam/coreslam.h
	gcc -O3 -c -Wall $(CFLAGS) ../coreslam/rmhc_filter.c

ziggrand.o: ../coreslam/ziggrand.c
	gcc -O3 -c -Wall $(CFLAGS) ../coreslam/ziggrand.c
	
install: libbreezyslam.$(LIBEXT)
	cp libbreezyslam.$(LIBEXT) $(LIBDIR)
	
doc:
	doxygen

clean:
	rm -rf  libbreezyslam.$(LIBEXT) *.o Documentation \#* *~
