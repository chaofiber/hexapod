#include <utils/timer.h>

/**
	Default constructor - resets the timer for the first time.
*/
Timer::Timer() {
#ifdef WIN32
	//first, get an idea of the frequency...
	DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), (DWORD_PTR)1);
	QueryPerformanceFrequency((LARGE_INTEGER *)&this->frequency);
	//frequency is in counts per second..., we want it to be counts per millisecond instead...
	countsPerMillisecond = this->frequency / 1000.0;
	SetThreadAffinityMask(GetCurrentThread(), oldmask);
#else
#endif
	restart();

}

/**
	This method resets the starting time. All the time Ellapsed function calls will use the reset start time for their time evaluations.
*/
void Timer::restart() {
#ifdef WIN32
	DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), (DWORD_PTR)1);
	QueryPerformanceCounter((LARGE_INTEGER *)&this->startTime);
	SetThreadAffinityMask(GetCurrentThread(), oldmask);
#else
	gettimeofday(&startTime, nullptr);
#endif
}

/**
	This method returns the time, in seconds, that has ellapsed since the timer was reset.
*/
double Timer::timeEllapsed() {
#ifdef WIN32
	long long int tempTime;
	//force the thread to run on CPU 0 because the QPC method is buggy
	DWORD_PTR oldmask = SetThreadAffinityMask(GetCurrentThread(), (DWORD_PTR)1);
	QueryPerformanceCounter((LARGE_INTEGER *)&tempTime);
	//let it run wild and free again
	SetThreadAffinityMask(GetCurrentThread(), oldmask);
	if (tempTime < startTime)
		return 0;
	return (tempTime - startTime) / (double)frequency;
#else
	struct timeval endTime;
	struct timeval interval_elapsed;
	gettimeofday(&endTime, 0);
	timeval_subtract(&interval_elapsed, &endTime, &startTime);
	return (double)interval_elapsed.tv_sec + (double)interval_elapsed.tv_usec / 1000000;
#endif
}

void Timer::wait(double t) {
	double currentT = timeEllapsed();
	while (timeEllapsed() - currentT < t);
}

