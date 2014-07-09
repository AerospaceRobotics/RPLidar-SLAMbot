USE_ODOMETRY = 1
USE_FILTER   = 1

#SIMD_FILE = coreslam_sisd

SIMD_FILE = coreslam_neon
SIMD_FLAG = -mfpu=neon

profile: breezytest
	./breezytest exp2 $(USE_ODOMETRY) $(USE_FILTER)
	gprof breezytest | less

breezytest: breezytest.cpp algorithms.o Random.o Scan.o Map.o Laser.o WheeledRobot.o coreslam.o $(SIMD_FILE).o ziggrand.o
	g++ -I../c++ -o breezytest breezytest.cpp  algorithms.o Random.o Scan.o Map.o Laser.o WheeledRobot.o coreslam.o  $(SIMD_FILE).o ziggrand.o -lm -pg

algorithms.o: ../c++/algorithms.cpp ../c++/algorithms.hpp ../c++/Laser.hpp ../c++/Position.hpp ../c++/Map.hpp \
              ../c++/Scan.hpp ../c++/Velocities.hpp ../c++/WheeledRobot.hpp ../c++/Random.hpp ../c/coreslam.h 
	g++ -g -c -Wall $(CFLAGS) -I../c -I../c++ ../c++/algorithms.cpp -pg

Random.o: ../c++/Random.cpp ../c++/Random.hpp 
	g++ -g -c -Wall $(CFLAGS) -I../c -I../c++ ../c++/Random.cpp -pg

Scan.o: ../c++/Scan.cpp ../c++/Scan.hpp ../c++/Velocities.hpp ../c++/Laser.hpp ../c/coreslam.h
	g++ -g -c -Wall $(CFLAGS) -I../c -I../c++ ../c++/Scan.cpp -pg

Map.o: ../c++/Map.cpp ../c++/Map.hpp ../c++/Position.hpp ../c++/Scan.hpp ../c/coreslam.h
	g++ -g -c -Wall $(CFLAGS) -I../c -I../c++ ../c++/Map.cpp -pg

Laser.o: ../c++/Laser.cpp ../c++/Laser.hpp ../c/coreslam.h
	g++ -g -c -Wall $(CFLAGS) -I../c -I../c++ ../c++/Laser.cpp -pg

WheeledRobot.o: ../c++/WheeledRobot.cpp ../c++/WheeledRobot.hpp 
	g++ -g -c -Wall $(CFLAGS) -I../c -I../c++ ../c++/WheeledRobot.cpp -pg

coreslam.o: ../c/coreslam.c ../c/coreslam.h ../c/coreslam_utils.h
	gcc -g -c -Wall $(CFLAGS) -I../c ../c/coreslam.c -pg

$(SIMD_FILE).o: ../c/$(SIMD_FILE).c ../c/coreslam.h ../c/coreslam_utils.h
	gcc -g -c -Wall $(CFLAGS) $(SIMD_FLAG) ../c/$(SIMD_FILE).c -pg

ziggrand.o: ../c/ziggrand.c
	gcc -g -c -Wall $(CFLAGS) ../c/ziggrand.c -pg
	
clean:
	rm -f  gmon.out breezytest *.o Documentation \#* *~

backup:
	cp *.cpp bak
	cp *.hpp bak
	cp *.c bak
	cp *.h bak
	cp Makefile bak
