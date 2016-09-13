#ifndef TEST_UTILS
#define TEST_UTILS

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <sys/stat.h>

#include "types.c"
#include "system.c"
#include "controls/servos.c"

struct TestInfo{
	const char* name;        // display name of this test
	const char* description; // brief description of test's duty (optional)
	void(*setup)();          // function pointer that performs any required setup prior to testing
	void(*teardown)();       // performs any needed cleanup
	int(*run)();             // testing function
};

static inline void Log(const char* format, int isGood, ...){
	char buf[1024] = {}, txt[1024] = {};
	va_list args;
    
	va_start(args, isGood);
	vsnprintf(buf, (size_t)sizeof(buf), format, args);
	va_end(args);

	if(isGood){
		sprintf(txt, "\t\033[0;32m%s\033[0m\n", buf);
	}
	else{
		sprintf(txt, "\t\033[1;31m%s\033[0m\n", buf);	
	}

	write(1, txt, strlen(txt) + 1);

	return;
}

#define TEST_BEGIN int main(int argc, const char* argv[]){\
	int ret = 0;\
	int testNumber = 1;\
	int totalTests = 1;\
	if(argc >= 3){\
		testNumber = atoi(argv[1]);\
		totalTests = atoi(argv[2]);\
	}\
	srandom(time(NULL));\
	struct TestInfo testInstance = {\
	
#define TEST_END };\
	printf("\e[1;33m[%s] (%d/%d)\033[0m\n", testInstance.name, testNumber + 1, totalTests);\
	if(testInstance.description) printf("%s\n", testInstance.description);\
	printf("\tBeginning setup...\n");\
	if(testInstance.setup) testInstance.setup();\
	printf("\tRunning testInstance...\n");\
	ret = testInstance.run();\
	printf("\tTearing down...\n");\
	if(testInstance.teardown) testInstance.teardown();\
	Log(ret ? "(%d) FAILED\n\n" : "(%d) PASSED\n\n", !ret, ret);\
	return ret;\
}\



#endif
